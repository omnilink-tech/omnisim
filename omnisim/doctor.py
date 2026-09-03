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

"""omnisim doctor — print ground-truth repo / runtime state for agents.

A new agent session (Claude Code, Codex, Cursor, Aider, …) that lands in
an OmniSim clone runs this on its first turn to know what is actually
true about this clone right now, instead of guessing from documentation.

AGENTS.md §3 (cold-start bootstrap) directs agents here.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import http.client
import importlib.util
import io
import json
import os
import re
import shutil
import socket
import subprocess
import sys
from pathlib import Path

from . import __version__
from .paths import DEMOS_WORLDS, REPO_ROOT, resolve_omnisim_binary


def invocation() -> str:
    """How to spell this CLI back to the user who just ran it.

    A Windows user with no system Python reaches the CLI through omnisim.bat,
    which uses the interpreter bundled beside the engine. Answering them with
    `python -m omnisim demo` hands them the one command they cannot run, so
    the launcher exports its own name and every "next command" echoes it.
    """
    return os.environ.get("OMNISIM_INVOKED_AS") or "python -m omnisim"


HARNESS_PORT = 6789
SUPERVISOR_PORT = 6790
CAPTURE_PORT = 6791

# The engine and libController must agree on the intern IPC pipe name. The engine
# exports a per-launch nonce (OmController::setProcessEnvironment) and names the pipe
# webots-<tmpId>-<nonce>-<robot>; libController falls back to the legacy
# webots-<tmpId>-<robot> when it cannot read the nonce (compute_socket_filename,
# src/controller/c/robot.c). An engine that HAS the nonce paired with a libController
# built BEFORE it therefore listen on two different pipes and never meet: every
# controller blocks forever in Robot(), the world finalises but never steps, and the
# run still exits 0. Probing for the symbol is exact, so this is a gate, not a guess.
_IPC_NONCE_TOKEN = b"OMNISIM_IPC_NONCE"

# lib name per platform; the engine links the same DLL/so that controllers load.
_CONTROLLER_LIB_NAMES = ("Controller.dll", "libController.so", "libController.dylib")

# The embedded physics runtime is IMPORTED FROM THE BUNDLE, not from src/. An
# ordinary rebuild does not re-stage it, so an edit to the source ships only
# after `bundle_newton_runtime.py --mode vendor` (AGENTS.md: measured 2026-08-09,
# a bundle 23 minutes older than the source and a full validation pass that
# proved nothing). Windows only -- Linux resolves the runtime from the system
# python3 and has no bundle.
RUNTIME_SOURCE = REPO_ROOT / "src" / "omnisim" / "physics" / "omnisim_newton_runtime.py"
RUNTIME_BUNDLE_REL = Path("newton-runtime") / "site-packages" / "omnisim_newton_runtime.py"
RUNTIME_VENDOR_FIX = "python scripts/packaging/bundle_newton_runtime.py --mode vendor"

WGPU_NATIVE_DLL = "wgpu_native.dll"
WGPU_SETUP_FIX = "bash scripts/dev/setup_wgpu_native.sh"
HOOKS_FIX = "bash scripts/dev/setup_hooks.sh"
ENGINE_PROCESS_NAMES = ("omnisim-bin.exe", "omnisim-bin")
_WORLD_ARG_RE = re.compile(r"(\S+\.(?:omniworld|wbt))(?=\s|$)", re.IGNORECASE)

# Hosted CI, as seen by the `gh` CLI. The local pre-push gate is a Windows
# smoke and cannot see the Linux renderer assertion or the provenance gate,
# so the private repo's CI sat RED for a day (2026-09-02: `linux-build`,
# `Licence and provenance`, `physics-runtime-check`) with every local check
# green and nobody looking. Advisory, never gating, and bounded: one
# `gh run list` (the ONLY network call doctor makes), one JSON parse, and at
# most CI_TIMEOUT_S of doctor's wall time -- offline, no gh, not logged in,
# not a GitHub remote all read `unknown (<reason>)` and move on.
CI_TIMEOUT_S = 1.5
CI_RUN_LIMIT = 12
CI_GREEN_CONCLUSIONS = ("success", "skipped")
_CI_JSON_FIELDS = "name,status,conclusion,headSha,createdAt,databaseId,url"
# https://github.com/o/r(.git), git@github.com:o/r(.git), ssh://git@github.com/o/r(.git)
_GITHUB_REMOTE_RE = re.compile(r"github\.com[:/]([^/\s]+)/([^/\s]+?)(?:\.git)?/?$", re.IGNORECASE)


def _git(*args: str) -> str | None:
    try:
        out = subprocess.check_output(
            ["git", *args], cwd=REPO_ROOT, stderr=subprocess.DEVNULL
        )
        return out.decode("utf-8", errors="replace").strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def _port_status(port: int, host: str = "127.0.0.1") -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.2)
    try:
        return "in-use" if sock.connect_ex((host, port)) == 0 else "free"
    finally:
        sock.close()


def _sha256(path: Path) -> str | None:
    try:
        h = hashlib.sha256()
        with path.open("rb") as fh:
            for block in iter(lambda: fh.read(1 << 20), b""):
                h.update(block)
        return h.hexdigest()
    except OSError:
        return None


def _runtime_bundle_status(source: Path, bundle: Path | None) -> dict:
    """Is the vendored physics runtime the same bytes as the source?

    Verdicts: ``ok`` (byte-identical), ``stale`` (differ AND the source is
    newer -- an edit that never shipped; FAIL), ``differs`` (differ but the
    bundle is newer -- vendored from another revision; WARN), ``absent`` (no
    bundle on this platform, or no engine), ``unknown`` (unreadable).
    Pure: takes paths, touches nothing but the two files.
    """
    info: dict = {"status": "absent", "source": str(source), "bundle": str(bundle) if bundle else None,
                  "source_sha256": None, "bundle_sha256": None, "detail": "", "fix": None}
    if not source.is_file():
        info["detail"] = "runtime source not in this tree"
        return info
    if bundle is None or not bundle.is_file():
        info["detail"] = ("no vendored runtime beside the engine" if bundle
                          else "no bundle on this platform (the runtime is resolved from the "
                               "system python3)")
        return info
    src_sha, bun_sha = _sha256(source), _sha256(bundle)
    info["source_sha256"], info["bundle_sha256"] = src_sha, bun_sha
    if src_sha is None or bun_sha is None:
        info["status"] = "unknown"
        info["detail"] = "could not read one of the two runtime copies"
        return info
    if src_sha == bun_sha:
        info["status"] = "ok"
        info["detail"] = f"bundle == source (sha256 {src_sha[:12]})"
        return info
    try:
        source_newer = source.stat().st_mtime > bundle.stat().st_mtime
    except OSError:
        source_newer = False
    info["source_newer"] = source_newer
    if source_newer:
        info["status"] = "stale"
        info["detail"] = (
            "the vendored runtime is STALE: src/omnisim/physics/omnisim_newton_runtime.py was "
            "edited after the last vendor, and the engine imports the BUNDLE -- every run and "
            "every test is exercising the OLD physics code"
        )
        info["fix"] = RUNTIME_VENDOR_FIX
    else:
        info["status"] = "differs"
        info["detail"] = (
            "the vendored runtime differs from the source but is NEWER -- vendored from another "
            "revision? Re-vendor to be sure which code the engine runs"
        )
        info["fix"] = RUNTIME_VENDOR_FIX
    return info


def _pillow_status() -> dict:
    """Pillow in THIS interpreter: the harness's /world/render_stats needs it (503 without)."""
    try:
        available = importlib.util.find_spec("PIL") is not None
    except (ImportError, ValueError):
        available = False
    return {
        "available": available,
        "interpreter": sys.executable,
        "detail": ("Pillow importable" if available else
                   "Pillow NOT importable in this interpreter -- the harness's "
                   "GET /world/render_stats returns 503 without it (screenshots still work)"),
        "fix": None if available else f"{sys.executable} -m pip install Pillow",
    }


def _wgpu_native_status(binary: str | None) -> dict:
    """wgpu is the ONLY renderer; on Windows it is a DLL beside the engine."""
    info: dict = {"status": "unknown", "path": None, "detail": "", "fix": None}
    if not binary:
        info["detail"] = "no engine binary, so no renderer to check"
        return info
    if os.name != "nt":
        info["detail"] = "checked on Windows only (the DLL layout); Linux links it at build time"
        return info
    dll = Path(binary).parent / WGPU_NATIVE_DLL
    info["path"] = str(dll)
    if dll.is_file():
        info["status"] = "present"
        info["detail"] = f"{WGPU_NATIVE_DLL} beside the engine"
    else:
        info["status"] = "absent"
        info["detail"] = (
            f"{WGPU_NATIVE_DLL} is NOT beside the engine -- this install has NO renderer "
            "(physics and controllers run; every screenshot and camera image is empty)"
        )
        info["fix"] = WGPU_SETUP_FIX
    return info


def _world_from_cmdline(cmdline: str) -> str | None:
    """The world path an engine was launched on, from its command line."""
    if not cmdline:
        return None
    m = _WORLD_ARG_RE.search(cmdline.replace('"', ""))
    return m.group(1) if m else None


def _parse_tasklist_csv(text: str) -> list[int]:
    """PIDs from `tasklist /FO CSV /NH` output (pure)."""
    pids: list[int] = []
    for row in csv.reader(io.StringIO(text)):
        if len(row) >= 2 and row[0].lower() in ENGINE_PROCESS_NAMES:
            try:
                pids.append(int(row[1]))
            except ValueError:
                continue
    return pids


def _parse_cim_json(text: str) -> list[dict]:
    """`Get-CimInstance ... | ConvertTo-Json` rows -> [{pid, cmdline, world}] (pure)."""
    text = text.strip()
    if not text:
        return []
    try:
        data = json.loads(text)
    except ValueError:
        return []
    rows = data if isinstance(data, list) else [data]
    out = []
    for r in rows:
        if not isinstance(r, dict):
            continue
        cmd = r.get("CommandLine") or ""
        out.append({"pid": r.get("ProcessId"), "cmdline": cmd, "world": _world_from_cmdline(cmd)})
    return out


def _parse_ps_output(text: str) -> list[dict]:
    """`ps -eo pid=,args=` rows naming the engine -> [{pid, cmdline, world}] (pure)."""
    out = []
    for line in text.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) != 2 or not parts[0].isdigit():
            continue
        cmd = parts[1]
        exe = cmd.split()[0].rsplit("/", 1)[-1] if cmd.split() else ""
        if exe in ENGINE_PROCESS_NAMES:
            out.append({"pid": int(parts[0]), "cmdline": cmd, "world": _world_from_cmdline(cmd)})
    return out


def _engine_processes() -> dict:
    """Running engines on this host, with PID and world. NEVER a kill list.

    Running engines coexist by design (AGENTS.md §3e). An engine you did not
    spawn is another session's live run, not an orphan: `taskkill /F` on it
    gives that session exit code 1 with nothing in its log, which reads exactly
    like an engine crash (measured 2026-08-29, five epochs lost).
    """
    info: dict = {"count": 0, "engines": [], "detail": "", "source": None}
    try:
        if os.name == "nt":
            # tasklist is ~0.1 s; the CIM query that yields command lines costs
            # ~0.6 s, so it is paid only when an engine is actually running.
            out = subprocess.run(
                ["tasklist", "/FI", "IMAGENAME eq omnisim-bin.exe", "/FO", "CSV", "/NH"],
                capture_output=True, text=True, timeout=5,
            )
            pids = _parse_tasklist_csv(out.stdout)
            info["source"] = "tasklist"
            engines = [{"pid": pid, "cmdline": None, "world": None} for pid in pids]
            if pids:
                ps = subprocess.run(
                    ["powershell", "-NoProfile", "-NonInteractive", "-Command",
                     "Get-CimInstance Win32_Process -Filter \"Name='omnisim-bin.exe'\" | "
                     "Select-Object ProcessId,CommandLine | ConvertTo-Json -Compress"],
                    capture_output=True, text=True, timeout=10,
                )
                rows = _parse_cim_json(ps.stdout)
                if rows:
                    engines = rows
                    info["source"] = "Get-CimInstance Win32_Process"
        else:
            out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True,
                                 timeout=5)
            engines = _parse_ps_output(out.stdout)
            info["source"] = "ps"
    except (OSError, subprocess.SubprocessError) as exc:
        info["detail"] = f"could not list processes ({exc.__class__.__name__})"
        return info
    info["engines"] = engines
    info["count"] = len(engines)
    info["detail"] = ("no omnisim-bin running" if not engines else
                      f"{len(engines)} omnisim-bin running -- do not kill engines you did not spawn")
    return info


def _http_get_json(port: int, path: str, connect_timeout: float = 0.2,
                   read_timeout: float = 1.0) -> tuple[int | None, dict | None]:
    """GET 127.0.0.1:<port><path> -> (status, json|None). Closed ports on this
    box take ~2 s to refuse (firewall drop), hence the short connect timeout."""
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=connect_timeout)
    try:
        conn.connect()
        if conn.sock is not None:
            conn.sock.settimeout(read_timeout)
        conn.request("GET", path, headers={"Accept": "application/json"})
        resp = conn.getresponse()
        raw = resp.read()
        try:
            body = json.loads(raw) if raw else None
        except ValueError:
            body = None
        return resp.status, body if isinstance(body, dict) else None
    except (OSError, http.client.HTTPException):
        return None, None
    finally:
        try:
            conn.close()
        except Exception:  # noqa: BLE001 -- closing must never raise
            pass


def _harness_world(state: dict | None) -> str | None:
    """The world a harness holds, from its /sim/state body (pure; key-tolerant)."""
    if not isinstance(state, dict):
        return None
    for key in ("world", "world_path", "current_world", "path"):
        v = state.get(key)
        if isinstance(v, str) and v:
            return v
        if isinstance(v, dict):
            for k2 in ("path", "world", "file"):
                if isinstance(v.get(k2), str) and v[k2]:
                    return v[k2]
    last = state.get("last_load")
    if isinstance(last, dict):
        for k2 in ("path", "world"):
            if isinstance(last.get(k2), str) and last[k2]:
                return last[k2]
    return None


def _harness_probe(port: int = HARNESS_PORT) -> dict:
    """When :6789 is in use, ask it what it is (only a harness answers /healthz)."""
    info: dict = {"port": port, "reachable": False, "healthz": None, "world": None,
                  "supervisor": None, "detail": ""}
    status, _ = _http_get_json(port, "/healthz")
    info["healthz"] = status
    if status != 200:
        info["detail"] = (f"port {port} is in use but did not answer GET /healthz -- "
                          "something other than a harness, or a harness still starting")
        return info
    info["reachable"] = True
    st_status, state = _http_get_json(port, "/sim/state")
    if st_status == 200 and state:
        info["world"] = _harness_world(state)
        sup = state.get("supervisor_connected")
        info["supervisor"] = sup if isinstance(sup, bool) else None
    info["detail"] = ("harness on " + info["world"]) if info["world"] else "harness up, no world loaded"
    return info


def _hooks_path_ok(value: str | None, repo_root: Path) -> bool:
    """Does `git config core.hooksPath` point at this clone's .githooks? (pure)"""
    if not value:
        return False
    v = Path(value.strip().strip('"'))
    if not v.is_absolute():
        v = repo_root / v
    try:
        return v.resolve() == (repo_root / ".githooks").resolve()
    except OSError:
        return False


def _git_hooks_status() -> dict:
    value = _git("config", "core.hooksPath")
    ok = _hooks_path_ok(value, REPO_ROOT)
    return {
        "hooks_path": value,
        "ok": ok,
        "detail": ("core.hooksPath -> .githooks" if ok else
                   "core.hooksPath is not this clone's .githooks -- the post-checkout orphan "
                   "purge and the pre-push smoke gate are not running"),
        "fix": None if ok else HOOKS_FIX,
    }


def _github_repo_from_remote(url: str | None) -> str | None:
    """`owner/repo` from an origin URL (https, ssh or scp-style); None otherwise (pure)."""
    if not url:
        return None
    m = _GITHUB_REMOTE_RE.search(url.strip())
    return f"{m.group(1)}/{m.group(2)}" if m else None


def _ci_classify(runs: list, head_sha: str | None) -> dict:
    """Newest run per workflow NAME -> a green verdict (pure).

    ``runs`` is what `gh run list --json name,status,conclusion,headSha,
    createdAt,databaseId,url` returns, in any order (sorted here, newest
    first). Green means every workflow's newest run concluded ``success`` or
    ``skipped``. A run still in flight has an empty conclusion and is NOT
    green -- it is listed under its status (``in_progress``, ``queued``),
    because "CI has not answered on this commit yet" is exactly what an agent
    about to trust a push needs to hear, not something to round up to OK.
    """
    rows = [r for r in runs if isinstance(r, dict) and r.get("name")]
    rows.sort(key=lambda r: str(r.get("createdAt") or ""), reverse=True)
    newest: dict[str, dict] = {}
    for r in rows:
        newest.setdefault(str(r["name"]), r)
    workflows: list[dict] = []
    non_green: list[dict] = []
    for name in sorted(newest):
        r = newest[name]
        conclusion = str(r.get("conclusion") or "")
        status = str(r.get("status") or "")
        wf = {
            "name": name,
            "status": status,
            "conclusion": conclusion,
            "head_sha": r.get("headSha"),
            "created_at": r.get("createdAt"),
            "url": r.get("url"),
            "id": r.get("databaseId"),
        }
        workflows.append(wf)
        if conclusion not in CI_GREEN_CONCLUSIONS:
            non_green.append({**wf, "label": conclusion or status or "unknown"})
    newest_sha = rows[0].get("headSha") if rows else None
    return {
        "workflows": workflows,
        "non_green": non_green,
        "green": (not non_green) if workflows else None,
        "green_count": len(workflows) - len(non_green),
        "total": len(workflows),
        "newest_sha": newest_sha,
        "on_head": bool(newest_sha and head_sha and newest_sha == head_sha),
    }


def _ci_behind(sha: str | None, head_sha: str | None) -> int | None:
    """How many commits HEAD is ahead of `sha` (0 when equal).

    None when git cannot place `sha` at all -- not fetched, rewritten away,
    or simply not an ancestor -- which the row reports as `not an ancestor`.
    """
    if not sha or not head_sha:
        return None
    if sha == head_sha:
        return 0
    out = _git("rev-list", "--count", f"{sha}..HEAD")
    if out is None or not out.isdigit():
        return None
    return int(out)


def _ci_reason(stderr: str) -> str:
    """One word for why `gh run list` failed, from its stderr (pure)."""
    s = (stderr or "").lower()
    if "auth" in s or "401" in s or "403" in s:
        return "auth"
    if any(k in s for k in ("dial tcp", "no such host", "connect", "network", "timeout", "tls")):
        return "offline"
    if "404" in s:
        return "no-repo"
    return "gh-error"


def _ci_finish(info: dict, runs: list, behind=None) -> dict:
    """Fold a parsed run list into a launched `info` (pure but for `behind`).

    `behind(sha, head_sha)` defaults to the git distance; tests pass a stub.
    """
    behind = behind or _ci_behind
    cls = _ci_classify(runs, info.get("head_sha"))
    info["available"] = True
    info.setdefault("reason", None)
    info.setdefault("detail", "")
    info["behind"] = None
    info["next"] = None
    info["workflows"] = cls["workflows"]
    info["non_green"] = cls["non_green"]
    info["green"] = cls["green"]
    info["green_count"] = cls["green_count"]
    info["total"] = cls["total"]
    info["newest_sha"] = cls["newest_sha"]
    if not cls["workflows"]:
        info["reason"] = "no-runs"
        info["detail"] = f"no workflow runs on {info.get('branch')} in the last {CI_RUN_LIMIT}"
        return info
    info["behind"] = behind(cls["newest_sha"], info.get("head_sha"))
    if cls["non_green"]:
        first = cls["non_green"][0]
        verb = "view" if first["status"] == "completed" else "watch"
        flag = " --log-failed" if verb == "view" else ""
        info["next"] = f"gh run {verb} {first['id']}{flag}  ({first['url']})"
    return info


def _ci_launch(head_sha: str | None, branch: str | None) -> dict:
    """Start the one `gh run list` call; `_ci_collect` reaps it.

    Split in two so `run()` can overlap gh's ~1 s round-trip with the local
    checks instead of paying it serially. Returns a terminal ``info`` (no
    ``_proc``) whenever there is nothing to ask: no gh on PATH, no origin,
    origin not on GitHub, detached HEAD.
    """
    info: dict = {
        "available": False, "repo": None, "branch": branch, "head_sha": head_sha,
        "workflows": [], "green": None, "reason": None, "detail": "",
        "newest_sha": None, "behind": None, "next": None,
    }
    gh = shutil.which("gh")
    if not gh:
        info["reason"], info["detail"] = "no-gh", "the gh CLI is not on PATH"
        return info
    remote = _git("remote", "get-url", "origin")
    if not remote:
        info["reason"], info["detail"] = "no-remote", "no `origin` remote (or no git)"
        return info
    repo = _github_repo_from_remote(remote)
    if not repo:
        info["reason"], info["detail"] = "not-github", f"origin is not on github.com ({remote})"
        return info
    info["repo"] = repo
    if not branch or branch in ("?", "HEAD"):
        info["reason"], info["detail"] = "detached", "not on a branch"
        return info
    env = {**os.environ, "GH_NO_UPDATE_NOTIFIER": "1", "GH_PROMPT_DISABLED": "1", "NO_COLOR": "1"}
    cmd = [gh, "run", "list", "--repo", repo, "--branch", branch,
           "--limit", str(CI_RUN_LIMIT), "--json", _CI_JSON_FIELDS]
    try:
        info["_proc"] = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL,
            cwd=REPO_ROOT, env=env,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        info["reason"], info["detail"] = "spawn-failed", f"could not start gh ({exc.__class__.__name__})"
    return info


def _ci_collect(info: dict) -> dict:
    """Reap the gh call started by `_ci_launch` within CI_TIMEOUT_S; never raises."""
    proc = info.pop("_proc", None)
    if proc is None:
        return info
    try:
        stdout, stderr = proc.communicate(timeout=CI_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.communicate(timeout=1)
        except Exception:  # noqa: BLE001 -- reaping a killed child must never raise
            pass
        info["reason"], info["detail"] = "timeout", f"gh did not answer within {CI_TIMEOUT_S:g} s"
        return info
    except (OSError, ValueError) as exc:
        info["reason"], info["detail"] = "gh-error", f"could not read gh ({exc.__class__.__name__})"
        return info
    out = (stdout or b"").decode("utf-8", errors="replace")
    err = (stderr or b"").decode("utf-8", errors="replace").strip()
    if proc.returncode != 0:
        info["reason"] = _ci_reason(err)
        info["detail"] = err.splitlines()[0] if err else f"gh exited {proc.returncode}"
        return info
    try:
        runs = json.loads(out)
    except ValueError:
        runs = None
    if not isinstance(runs, list):
        info["reason"], info["detail"] = "not-json", "gh printed something other than a JSON run list"
        return info
    return _ci_finish(info, runs)


def _ci_status(head_sha: str | None = None, branch: str | None = None) -> dict:
    """The serial form: launch + collect. `run()` overlaps the two instead."""
    return _ci_collect(_ci_launch(head_sha, branch))


def _ci_row_lines(info: dict) -> list[str]:
    """The `ci` row for the text report (pure). WARN when anything is not green."""
    if not info.get("available"):
        return [f"ci          unknown ({info.get('reason') or 'unavailable'})"]
    n, k = info.get("total", 0), info.get("green_count", 0)
    if n == 0:
        return ["ci          unknown (no-runs)"]
    newest, head = info.get("newest_sha") or "", info.get("head_sha") or ""
    where = f"on {newest[:7] or '?'}"
    if newest != head:
        behind = info.get("behind")
        if behind is None:
            where += " (not an ancestor of HEAD)"
        elif behind > 0:
            where += f", behind by {behind} commit{'s' if behind != 1 else ''}"
        else:
            where += ", ahead of HEAD"
    if info.get("green"):
        return [f"ci          {k}/{n} workflows green {where}"]
    names = ", ".join(f"{w['name']}: {w['label']}" for w in info.get("non_green", []))
    lines = [f"ci          WARN: {k}/{n} workflows green {where} ({names})"]
    if info.get("next"):
        lines.append(f"            next: {info['next']}")
    return lines


def _ffmpeg_status() -> dict:
    path = shutil.which("ffmpeg")
    return {
        "path": path,
        "detail": (f"on PATH at {path}" if path else
                   "not on PATH -- capture (/capture/sequence) and cinema need it to encode"),
    }


def _worlds() -> list[str]:
    if not DEMOS_WORLDS.is_dir():
        return []
    # Recursive: most demos live in subdirs (showcase/, flagship/, chat/,
    # ...). A top-level-only glob reported 2 of 73 worlds and made fresh
    # clones look empty.
    # Dot-prefixed names are the harness's own scratch copies
    # (.harness_*.omniworld, gitignored). They sort first, so an unfiltered
    # listing showed a newcomer five temp files as the sample of "your worlds"
    # and inflated the count by 26 on a box where the harness had ever run.
    def _authored(path: Path) -> bool:
        rel = path.relative_to(DEMOS_WORLDS)
        return not any(part.startswith(".") for part in rel.parts)

    return sorted(
        p.relative_to(DEMOS_WORLDS).as_posix()
        for p in [*DEMOS_WORLDS.rglob("*.omniworld"), *DEMOS_WORLDS.rglob("*.wbt")]
        if _authored(p)
    )


def _physics_runtime(binary: str | None) -> dict:
    """Is there a physics backend at all?

    Newton has been the ONLY backend since `bdc02139` deleted ODE, so "absent"
    is not a degraded mode -- it is an install with no dynamics whatsoever:
    nothing falls, nothing collides, no grasp holds, and the engine still
    exits 0. README tells the user `doctor` reports this, so it must.

    It cannot be one existence test, because the runtime is resolved
    differently per platform:

    * Windows -- the release BUNDLES it next to the binary, and the embedded
      interpreter loads it through `python312._pth`. Presence is a file test.
    * Linux   -- there is no bundle. The embedded interpreter resolves the
      SYSTEM `python3` (and ignores venvs), so the wheels must be importable
      from that interpreter. Probed with `importlib.util.find_spec`, which
      resolves without executing the module (importing `warp` initialises
      CUDA and costs seconds).
    """
    info: dict = {"status": "unknown", "source": None, "detail": "", "fix": None}
    if not binary:
        info["detail"] = "no engine binary, so no runtime to check"
        return info

    if os.name == "nt":
        info["source"] = "bundle"
        root = Path(binary).parent / "newton-runtime" / "site-packages"
        missing = [m for m in ("warp", "newton", "mujoco") if not (root / m).exists()]
        if not missing:
            info["status"] = "present"
            info["detail"] = "Newton runtime bundled next to the engine"
        else:
            info["status"] = "absent"
            info["detail"] = (
                "Newton runtime NOT bundled (missing " + ", ".join(missing) + ") -- "
                "this install has NO physics at all"
            )
            info["fix"] = "make -C src/omnisim bundle-newton-runtime"
        return info

    # Probe the interpreter the BINARY LINKS, not a bare `python3`. They can
    # legitimately differ: on Ubuntu 22.04 the system python3 is 3.10 (where
    # newton 1.5.0 raises at ModelBuilder()), so the bootstrap installs 3.12
    # from deadsnakes and the engine embeds that -- probing python3 there
    # would report the wheels missing on a perfectly healthy install.
    py = "python3"
    try:
        import re as _re
        ldd = subprocess.run(["ldd", binary], capture_output=True, text=True, timeout=15)
        m = _re.search(r"libpython(3\.\d+)", ldd.stdout)
        if m and shutil.which("python" + m.group(1)):
            py = "python" + m.group(1)
    except (OSError, subprocess.SubprocessError):
        pass
    info["source"] = f"linked-interpreter ({py})"
    probe = (
        "import importlib.util as u;"
        "print(','.join(m for m in ('warp','newton','mujoco') if u.find_spec(m) is None))"
    )
    try:
        out = subprocess.run(
            [py, "-c", probe],
            capture_output=True, text=True, timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        info["detail"] = f"could not run `{py}` to check the physics runtime"
        return info
    if out.returncode != 0:
        info["detail"] = f"`{py} -c` failed, so the physics runtime is unverified"
        return info
    missing = [m for m in out.stdout.strip().split(",") if m]
    if not missing:
        info["status"] = "present"
        info["detail"] = f"physics wheels importable from {py}"
    else:
        info["status"] = "absent"
        info["detail"] = (
            py + " cannot import " + ", ".join(missing) + " -- "
            "this install has NO physics at all"
        )
        info["fix"] = (
            py + " -m pip install warp-lang newton mujoco mujoco-warp "
            "(the interpreter the engine links -- it ignores venvs)"
        )
    return info


def _python_runtime() -> dict:
    """The interpreter versions that decide whether physics and controllers work.

    Two different interpreters matter and they are not always the same one:
    this CLI's, and the `python`/`python3` on PATH -- which the engine spawns
    for every Python controller, and which on Linux also supplies the physics
    wheels. newton 1.5.0 raises `TypeError: Union[arg, ...]` at
    `ModelBuilder()` on CPython 3.10, so a 3.10 interpreter in the physics
    role is a world that loads and never moves.
    """
    info: dict = {
        "cli": "%d.%d.%d" % sys.version_info[:3],
        "spawned": None,
        "spawned_name": None,
        "too_old": False,
    }
    for name in ("python3", "python") if os.name != "nt" else ("python", "python3"):
        exe = shutil.which(name)
        if not exe:
            continue
        try:
            out = subprocess.run(
                [exe, "-c", "import sys;print('%d.%d.%d' % sys.version_info[:3])"],
                capture_output=True, text=True, timeout=15,
            )
        except (OSError, subprocess.SubprocessError):
            continue
        if out.returncode == 0 and out.stdout.strip():
            info["spawned"] = out.stdout.strip()
            info["spawned_name"] = name
            break
    for ver in (info["cli"], info["spawned"]):
        if not ver:
            continue
        parts = ver.split(".")
        if (int(parts[0]), int(parts[1])) < (3, 11):
            info["too_old"] = True
    return info


def _controller_lib() -> Path | None:
    lib_dir = REPO_ROOT / "lib" / "controller"
    for name in _CONTROLLER_LIB_NAMES:
        candidate = lib_dir / name
        if candidate.is_file():
            return candidate
    return None


def _has_token(path: Path, token: bytes, chunk: int = 1 << 20) -> bool | None:
    """Stream-scan a binary for `token`. None when the file can't be read."""
    try:
        overlap = len(token) - 1
        tail = b""
        with path.open("rb") as fh:
            while True:
                block = fh.read(chunk)
                if not block:
                    return False
                if token in tail + block:
                    return True
                tail = block[-overlap:] if overlap else b""
    except OSError:
        return None


def _build_provenance() -> dict:
    """Engine vs libController compatibility — the stale-lib hang, caught cheaply."""
    engine_path = resolve_omnisim_binary()
    engine = Path(engine_path) if engine_path else None
    lib = _controller_lib()

    info: dict = {
        "engine": str(engine) if engine else None,
        "controller_lib": str(lib) if lib else None,
        "verdict": "unknown",
        "detail": "",
    }
    if engine is None or lib is None:
        missing = "engine binary" if engine is None else "libController"
        info["detail"] = f"{missing} not found -- build the simulator"
        return info

    info["engine_mtime"] = int(engine.stat().st_mtime)
    info["controller_lib_mtime"] = int(lib.stat().st_mtime)

    engine_nonce = _has_token(engine, _IPC_NONCE_TOKEN)
    lib_nonce = _has_token(lib, _IPC_NONCE_TOKEN)
    info["engine_ipc_nonce"] = engine_nonce
    info["controller_lib_ipc_nonce"] = lib_nonce

    if engine_nonce is None or lib_nonce is None:
        info["detail"] = "could not read one of the binaries"
        return info

    if engine_nonce and not lib_nonce:
        info["verdict"] = "incompatible"
        info["detail"] = (
            "libController predates the engine's IPC nonce: controllers will hang "
            "forever in Robot() and the sim will never step (the run still exits 0). "
            "Rebuild: make -C src/controller/c release"
        )
        return info

    info["verdict"] = "ok"
    stale_s = info["engine_mtime"] - info["controller_lib_mtime"]
    if stale_s > 86400:
        info["detail"] = (
            f"libController is {stale_s // 86400}d older than the engine -- compatible "
            "on the IPC nonce, but rebuild if controllers misbehave"
        )
    return info


def _env_landmines() -> list[str]:
    """Env-var conditions that are latent traps but do NOT stop a run here.

    Deliberately conservative: only flag a condition that is unambiguously wrong
    (a var pointing at a path that does not exist), never one that is merely
    unusual (a var pointing somewhere else, an unset var). A gate that false-fires
    on a working install is worse than no gate, so these stay advisory.
    """
    out: list[str] = []
    webots_home = os.environ.get("WEBOTS_HOME")
    if webots_home and not Path(webots_home).is_dir():
        # Windows keeps a machine-level WEBOTS_HOME after Webots is uninstalled.
        # Since 2026-09-02 nothing ADOPTS a non-existent one: resources/Makefile.include,
        # resources/Makefile.os.include and qt_utils/StandardPaths.cpp all check the
        # directory exists first and otherwise behave as if the variable were unset.
        # So this is no longer a landmine, only clutter -- but it stays reported,
        # because a variable naming a path that is not there always misleads a reader,
        # and the fix is one command.
        removal = (
            "[Environment]::SetEnvironmentVariable('WEBOTS_HOME', $null, 'Machine')  "
            "# elevated PowerShell; use 'User' scope if it is set there"
            if os.name == "nt"
            else "remove the WEBOTS_HOME export from your shell profile"
        )
        out.append(
            f"WEBOTS_HOME='{webots_home}' is set but does not exist (stale, typically an "
            f"uninstalled Webots). Nothing reads it any more when it does not exist -- the "
            f"build and the robot windows ignore an absent path and use OMNISIM_HOME -- so "
            f"this is clutter, not a fault. Clear it with: {removal}"
        )
    return out


def _coherence(build: dict, physics: dict | None = None,
               runtime_bundle: dict | None = None) -> dict:
    """Tier 0 -- is this install even wired coherently enough to run at all?

    The cross-machine determinism tiers (docs/developer/cross-machine-determinism.md)
    all ask whether a run BEHAVES the same: physics, numerics, solver bands. None
    can catch a stale DLL, because that failure produces NO tick to compare -- the
    world finalises and then every controller blocks forever in Robot(). This is the
    missing layer below them: coherence, not behaviour.

    Consumes ``_build_provenance()`` output (no re-probe) plus cheap env checks and
    splits problems into:

    - ``fatal``    -- the install cannot be trusted to run a controller-driven world;
                      ``--strict`` exits non-zero.
    - ``advisory`` -- real, but a run still proceeds here; printed, never gates.
    """
    fatal: list[str] = []
    advisory: list[str] = []

    verdict = build.get("verdict")
    if verdict == "incompatible":
        # Exact symbol probe, not a heuristic: the engine HAS the IPC nonce and the
        # lib does NOT, so they name different pipes and never meet. Definitive.
        fatal.append("engine/libController ABI mismatch (IPC nonce): " + build.get("detail", ""))
    elif build.get("engine") is None:
        # No binary -> literally nothing to run. 100% reliable (a file exists or not).
        fatal.append("engine binary not found -- nothing to run (build the simulator first)")
    elif build.get("controller_lib") is None:
        # No libController -> every controller process fails to load its API.
        fatal.append("libController not found -- controllers cannot load their API")
    elif build.get("engine_ipc_nonce") is None or build.get("controller_lib_ipc_nonce") is None:
        # A binary existed but could not be read (OSError). We cannot PROVE
        # incoherence, so we must not fail on it -- unproven != broken.
        advisory.append("could not read a binary to verify the IPC nonce -- coherence unproven")

    # Physics. Newton is the ONLY backend, so its absence is not a degraded
    # mode -- it is an install where nothing falls, while the engine still
    # exits 0. README promises doctor reports this, so it is FATAL, not a note.
    if physics and physics.get("status") == "absent":
        fatal.append(physics["detail"] + (" -- fix: " + physics["fix"] if physics.get("fix") else ""))

    # A STALE vendored runtime is a run of the wrong physics code with a green
    # build: exact (two sha256 values differ and the source is the newer file),
    # and one command fixes it. `differs` (bundle newer) stays advisory.
    if runtime_bundle and runtime_bundle.get("status") == "stale":
        fatal.append(runtime_bundle["detail"] + " -- fix: " + runtime_bundle["fix"])
    elif runtime_bundle and runtime_bundle.get("status") == "differs":
        advisory.append(runtime_bundle["detail"] + " -- " + runtime_bundle["fix"])

    advisory.extend(_env_landmines())

    return {"ok": not fatal, "fatal": fatal, "advisory": advisory}


def run(argv: list[str]) -> int:
    # Never crash on output: on Windows stdout is cp1252 when piped, and a recent commit
    # subject, a world name or a path with a non-cp1252 character (measured 2026-09-02:
    # a subject containing U+2264) turned the whole doctor run into a UnicodeEncodeError
    # traceback and exit 1 -- the pre-push hook then blocks the push on a false FAIL.
    for _stream in (sys.stdout, sys.stderr):
        try:
            _stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    parser = argparse.ArgumentParser(
        prog="omnisim doctor",
        description="Report ground-truth repo and runtime state.",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of text.")
    parser.add_argument(
        "--fingerprint",
        action="store_true",
        help="Also capture the install config fingerprint (resolved physics "
        "backend, Newton-runtime presence, GPU, fingerprint_id).",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Deprecated alias -- kept for .githooks/pre-push and existing CI. "
        "Plain `doctor` now exits non-zero on a blocking problem, so this flag "
        "no longer changes anything.",
    )
    args = parser.parse_args(argv)

    fingerprint = None
    if args.fingerprint:
        try:
            from .conformance import collect_fingerprint
            fingerprint = collect_fingerprint(scrub_paths=True)
        except Exception as exc:  # never let the fingerprint break doctor
            fingerprint = {"error": f"fingerprint failed: {exc}"}

    branch = _git("rev-parse", "--abbrev-ref", "HEAD") or "?"
    commit = _git("rev-parse", "--short", "HEAD") or "?"
    recent = _git("log", "-3", "--format=%h %s") or ""
    # Started first and reaped last: the gh round-trip (~1 s) overlaps the
    # local checks below instead of adding to them. One subprocess, bounded.
    ci = _ci_launch(_git("rev-parse", "HEAD"), branch)
    webots = resolve_omnisim_binary()
    worlds = _worlds()

    ports = {
        "harness": _port_status(HARNESS_PORT),
        "supervisor": _port_status(SUPERVISOR_PORT),
        "capture": _port_status(CAPTURE_PORT),
    }
    build = _build_provenance()
    physics = _physics_runtime(webots)
    python = _python_runtime()
    runtime_bundle = _runtime_bundle_status(
        RUNTIME_SOURCE,
        (Path(webots).parent / RUNTIME_BUNDLE_REL) if (webots and os.name == "nt") else None,
    )
    wgpu = _wgpu_native_status(webots)
    pillow = _pillow_status()
    hooks = _git_hooks_status()
    ffmpeg = _ffmpeg_status()
    engines = _engine_processes()
    harness = _harness_probe(HARNESS_PORT) if ports["harness"] == "in-use" else None
    ci = _ci_collect(ci)
    coherence = _coherence(build, physics, runtime_bundle)
    # A fatal coherence problem means the install cannot run a controller-driven
    # world. Reporting that and exiting 0 made every caller -- a script, a CI
    # lane, an agent branching on $? -- read a broken install as a pass, so the
    # exit code now follows the verdict. --strict is kept as an explicit alias
    # for callers (.githooks/pre-push) that already ask for gate semantics.
    exit_code = 0 if coherence["ok"] else 1

    if args.json:
        print(
            json.dumps(
                {
                    "omnisim_version": __version__,
                    "git_branch": branch,
                    "git_commit": commit,
                    "omnisim_binary": webots,
                    "webots_binary": webots,  # legacy alias for tools that still read the old key
                    "build": build,
                    "physics": physics,
                    "python": python,
                    "runtime_bundle": runtime_bundle,
                    "wgpu_native": wgpu,
                    "pillow": pillow,
                    "git_hooks": hooks,
                    "ci": ci,
                    "ffmpeg": ffmpeg,
                    "engine_processes": engines,
                    "harness": harness,
                    "coherence": coherence,
                    "strict": args.strict,
                    "ports": ports,
                    "worlds_dir": str(DEMOS_WORLDS),
                    "worlds": worlds,
                    "recent_commits": recent.splitlines(),
                    "fingerprint": fingerprint,
                },
                indent=2,
            )
        )
        return exit_code

    print(f"omnisim     {__version__}")
    if commit == "?":
        # A packaged install ships no .git, so `? @ ?` was the first line the
        # primary README audience ever saw. Name the build instead.
        print(f"install     packaged (no git metadata) at {REPO_ROOT}")
    else:
        print(f"git         {branch} @ {commit}")
    if webots:
        print(f"binary      {webots}")
    else:
        print("binary      NOT FOUND -- this tree has no simulator to run")
        print("            build:  build_omni.bat                     (Windows)")
        print("            build:  bash scripts/install/linux_bootstrap.sh  (Linux)")
        print("            then:   make -C src/omnisim bundle-newton-runtime")
    # ASCII only: doctor must survive a cp1252 Windows console. A diagnostic that
    # UnicodeEncodeErrors while reporting the fault is worse than no diagnostic.
    if build["verdict"] == "incompatible":
        print(f"build       FAIL: ENGINE/libController MISMATCH -- {build['detail']}")
    elif build["verdict"] == "unknown":
        print(f"build       ?  {build['detail']}")
    elif build["detail"]:
        print(f"build       WARN: {build['detail']}")
    else:
        print("build       engine + libController compatible")
    # Physics is the check a newcomer most needs and could least guess at:
    # Newton is the only backend, so "absent" means nothing in any world will
    # ever move, and the engine reports that by exiting 0.
    if physics["status"] == "present":
        print(f"physics     Newton runtime OK ({physics['detail']})")
    elif physics["status"] == "absent":
        print(f"physics     FAIL: {physics['detail']}")
        if physics.get("fix"):
            print(f"            fix:  {physics['fix']}")
    else:
        print(f"physics     ?  {physics['detail']}")
    # The vendored runtime is what the engine RUNS; the source is what you
    # edited. When they differ the tests you just ran validated nothing.
    if runtime_bundle["status"] == "ok":
        print(f"runtime     {runtime_bundle['detail']}")
    elif runtime_bundle["status"] == "stale":
        print(f"runtime     FAIL: {runtime_bundle['detail']}")
        print(f"            fix:  {runtime_bundle['fix']}")
    elif runtime_bundle["status"] == "differs":
        print(f"runtime     WARN: {runtime_bundle['detail']}")
        print(f"            fix:  {runtime_bundle['fix']}")
    else:
        print(f"runtime     {runtime_bundle['detail']}")
    if wgpu["status"] == "present":
        print(f"renderer    {wgpu['detail']}")
    elif wgpu["status"] == "absent":
        print(f"renderer    WARN: {wgpu['detail']}")
        print(f"            fix:  {wgpu['fix']}")
    py_line = f"python      {python['cli']} (this CLI)"
    if python["spawned"] and python["spawned"] != python["cli"]:
        py_line += f"   {python['spawned_name']} on PATH: {python['spawned']}"
    elif not python["spawned"]:
        py_line += "   WARN: no python/python3 on PATH -- every Python controller will fail to start"
    print(py_line)
    if python["too_old"] and physics["status"] != "present":
        # Suppressed when the physics probe already found a working (linked)
        # interpreter: on Ubuntu 22.04 + deadsnakes the CLI runs on 3.10 by
        # design while the engine embeds 3.12, and warning about it would
        # read as a fault on a healthy install.
        print("            WARN: newton 1.5.0 raises at ModelBuilder() on CPython 3.10 --")
        print("                  use 3.12 for the interpreter that supplies physics.")
    # Coherence advisories are things NOT already on the build line above (env
    # landmines, unreadable-binary). Shown always -- they are useful info -- but
    # they never change the exit code.
    for note in coherence["advisory"]:
        print(f"warn        {note}")
    if pillow["available"]:
        print("pillow      importable (harness /world/render_stats will answer)")
    else:
        print(f"pillow      WARN: {pillow['detail']}")
        print(f"            fix:  {pillow['fix']}")
    if hooks["ok"]:
        print(f"hooks       {hooks['detail']}")
    else:
        print(f"hooks       WARN: {hooks['detail']}")
        print(f"            fix:  {hooks['fix']}")
    # Hosted CI. The pre-push smoke is local and Windows-only, so a red Linux
    # or provenance lane is invisible here unless someone asks -- this asks.
    for line in _ci_row_lines(ci):
        print(line)
    print(f"ffmpeg      INFO: {ffmpeg['detail']}")
    if engines["count"] == 0:
        print(f"engines     {engines['detail']}")
    else:
        print(f"engines     {engines['detail']}")
        for e in engines["engines"]:
            world = e.get("world") or "(world not visible on the command line)"
            print(f"              PID {e.get('pid')}  {world}")
    print("ports")
    if harness is None:
        print(f"  6789 harness     {ports['harness']}")
    else:
        print(f"  6789 harness     in-use -- {harness['detail']}")
    print(f"  6790 supervisor  {ports['supervisor']}")
    print(f"  6791 capture     {ports['capture']}")
    rel_worlds = DEMOS_WORLDS.relative_to(REPO_ROOT) if DEMOS_WORLDS.exists() else Path("(missing)")
    print(f"worlds      {len(worlds)} in {rel_worlds}")
    if worlds:
        head = ", ".join(worlds[:5])
        more = f" (+{len(worlds) - 5} more)" if len(worlds) > 5 else ""
        print(f"            {head}{more}")
    if recent:
        print("recent")
        for line in recent.splitlines():
            print(f"  {line}")
    if fingerprint is not None:
        from .conformance.report import format_fingerprint
        print(format_fingerprint(fingerprint))
    # THE VERDICT. This used to be printed only under --strict, so the default
    # report -- the one README, BETA and SUPPORT all tell a new user to run --
    # ended on a git log and left "am I OK to proceed?" unanswered. It is the
    # first question the command exists to answer, so it is always answered,
    # and it always names the next command.
    print()
    if coherence["ok"]:
        cli = invocation()
        print("VERDICT     READY.  Next:")
        print("              %-26s # see a robot move" % (cli + " demo"))
        print("              %-26s # the full catalogue" % (cli + " demos"))
    else:
        n = len(coherence["fatal"])
        print(f"VERDICT     NOT READY -- {n} blocking problem{'s' if n != 1 else ''}:")
        for i, msg in enumerate(coherence["fatal"], 1):
            print(f"  [{i}] {msg}")
        print("            Re-run `python -m omnisim doctor` after fixing.")
    return exit_code
