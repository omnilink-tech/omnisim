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

"""The ``shell`` tool set: the byte-identical cross-simulator baseline.

Four tools, and the hard rule is that **nothing here may mention a
simulator** -- not a product name, not a path, not a port, not a file
extension. The definitions are hashed (``tools_sha256``) and that hash is what
makes "we gave every simulator the same interface" checkable rather than
claimed. If a description ever needs to say "OmniSim", it belongs in
``shell_plus_tools``, not here.

The *implementations* are free to differ per simulator (different shell binary,
different container); only the definitions are frozen. That asymmetry is the
point: ``handler`` is deliberately outside every hash.

Isolation lives in :mod:`agentbench.runner.isolation`: the working directory is
the run's scratch dir, the environment is an allowlist, writes are confined to
scratch, and reads of the benchmark's own package are refused. ``run_shell``
cannot be confined that way -- see that module's honest note about what a bare
host cannot enforce.
"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

from agentbench.common.paths import engine_launch
from agentbench.runner.tools import ToolResult
from agentbench.runner.tools.manifest import ToolSet, ToolSpec

# How much of a tool result the model is shown. The trace keeps everything.
MODEL_TEXT_LIMIT = 20_000
DEFAULT_READ_LINES = 400
DEFAULT_DIR_ENTRIES = 200


def _clip(text, limit=MODEL_TEXT_LIMIT):
    if len(text) <= limit:
        return text
    head = text[: limit - 400]
    tail = text[-200:]
    return ("%s\n...[%d characters omitted from this view; the full result is "
            "in the run trace]...\n%s" % (head, len(text) - limit + 200, tail))


# --------------------------------------------------------------------------
# Handlers. Signature: (args: dict, timeout_s: float) -> ToolResult
# --------------------------------------------------------------------------

def _make_run_shell(sandbox):
    def run_shell(args, timeout_s):
        command = args.get("command")
        if not isinstance(command, str) or not command.strip():
            return ToolResult(text="error: 'command' must be a non-empty "
                                   "string", is_error=True)
        cwd = sandbox.scratch_dir
        if args.get("cwd"):
            cwd = sandbox.guard.resolve(args["cwd"], sandbox.scratch_dir)
            why = sandbox.guard.check_read(cwd)
            if why:
                return ToolResult(text="error: cwd refused: %s" % why,
                                  is_error=True)
        # A hidden (read-denied) cwd takes the same branch as a missing one:
        # hidden paths must be indistinguishable from nonexistent paths.
        if sandbox.guard.is_hidden(cwd) or not Path(cwd).is_dir():
            return ToolResult(text="error: cwd does not exist: %s" % cwd,
                              is_error=True)
        want = float(args.get("timeout_s") or timeout_s)
        limit = max(1.0, min(want, timeout_s))
        argv = list(sandbox.shell_argv) + [command]
        t0 = time.perf_counter()
        timed_out = False
        # Popen + tree-kill rather than subprocess.run(timeout=...): killing
        # only the shell leaves its children alive holding the pipes open, and
        # `communicate` then blocks until THEY exit. Measured: a `sleep 5` with
        # a 1 s timeout returned after 5.4 s, which would let one runaway tool
        # call walk straight through the run's deadline.
        popen_kw = {}
        if os.name != "nt":
            popen_kw["start_new_session"] = True   # own pgid, so killpg lands
        proc = subprocess.Popen(argv, cwd=str(cwd), env=sandbox.env(),
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True,
                                errors="replace", **popen_kw)
        try:
            out, err = proc.communicate(timeout=limit)
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
            rc = None
            engine_launch.kill_tree(proc)
            try:
                out, err = proc.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                out, err = "", ""
            err = (err or "") + ("\n[killed after %.1fs: the command "
                                 "exceeded its timeout]" % limit)
        dt = time.perf_counter() - t0
        data = {"exit_code": rc, "timed_out": timed_out,
                "duration_s": round(dt, 3), "cwd": str(cwd),
                "stdout": out, "stderr": err}
        view = "exit_code: %s%s\n" % (rc, "  (timed out)" if timed_out else "")
        if out:
            view += "--- stdout ---\n%s\n" % out
        if err:
            view += "--- stderr ---\n%s\n" % err
        if not out and not err:
            view += "(no output)\n"
        return ToolResult(text=_clip(view), data=data,
                          is_error=bool(timed_out or (rc not in (0, None))))
    return run_shell


def _make_read_file(sandbox):
    def read_file(args, timeout_s):
        p = sandbox.guard.resolve(args.get("path", ""), sandbox.scratch_dir)
        # Hidden first, and merged into the not-a-file branch: a read-denied
        # path must return the BYTE-IDENTICAL error a nonexistent path
        # returns, so the agent cannot infer what is being hidden from the
        # shape of the refusal (docs-ablation cell; tested by
        # test_docs_ablation.py).
        if sandbox.guard.is_hidden(p):
            return ToolResult(text="error: not a file: %s" % p, is_error=True)
        why = sandbox.guard.check_read(p)
        if why:
            return ToolResult(text="error: refused: %s" % why, is_error=True)
        if not p.is_file():
            return ToolResult(text="error: not a file: %s" % p, is_error=True)
        try:
            text = p.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            return ToolResult(text="error: %s" % exc, is_error=True)
        lines = text.splitlines()
        start = max(1, int(args.get("start_line") or 1))
        count = int(args.get("max_lines") or DEFAULT_READ_LINES)
        chunk = lines[start - 1: start - 1 + count]
        body = "\n".join("%6d\t%s" % (start + i, ln)
                         for i, ln in enumerate(chunk))
        header = ("%s  (lines %d-%d of %d)\n"
                  % (p, start, start + len(chunk) - 1 if chunk else start,
                     len(lines)))
        return ToolResult(text=_clip(header + body),
                          data={"path": str(p), "total_lines": len(lines),
                                "start_line": start, "returned": len(chunk)})
    return read_file


def _make_write_file(sandbox):
    def write_file(args, timeout_s):
        p = sandbox.guard.resolve(args.get("path", ""), sandbox.scratch_dir)
        why = sandbox.guard.check_write(p)
        if why:
            return ToolResult(text="error: refused: %s" % why, is_error=True)
        content = args.get("content")
        if not isinstance(content, str):
            return ToolResult(text="error: 'content' must be a string",
                              is_error=True)
        try:
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content, encoding="utf-8")
        except OSError as exc:
            return ToolResult(text="error: %s" % exc, is_error=True)
        n = len(content.encode("utf-8"))
        return ToolResult(
            text="wrote %d bytes to %s (%d lines)"
                 % (n, p, content.count("\n") + 1),
            data={"path": str(p), "bytes": n,
                  "lines": content.count("\n") + 1})
    return write_file


def _make_list_dir(sandbox):
    def list_dir(args, timeout_s):
        p = sandbox.guard.resolve(args.get("path") or ".",
                                  sandbox.scratch_dir)
        # Same indistinguishability rule as read_file: a hidden directory
        # lists exactly like a nonexistent one.
        if sandbox.guard.is_hidden(p):
            return ToolResult(text="error: not a directory: %s" % p,
                              is_error=True)
        why = sandbox.guard.check_read(p)
        if why:
            return ToolResult(text="error: refused: %s" % why, is_error=True)
        if not p.is_dir():
            return ToolResult(text="error: not a directory: %s" % p,
                              is_error=True)
        cap = int(args.get("max_entries") or DEFAULT_DIR_ENTRIES)
        it = sorted(p.rglob("*")) if args.get("recursive") else sorted(
            p.iterdir())
        # Hidden entries are filtered BEFORE the count: `total_seen` including
        # them would leak their existence through arithmetic.
        it = [c for c in it if not sandbox.guard.is_hidden(c)]
        rows, n = [], 0
        for child in it:
            if n >= cap:
                break
            try:
                is_dir = child.is_dir()
                size = 0 if is_dir else child.stat().st_size
            except OSError:
                continue
            rows.append({"name": str(child.relative_to(p)),
                         "dir": is_dir, "bytes": size})
            n += 1
        view = "%s  (%d entr%s%s)\n" % (
            p, len(rows), "y" if len(rows) == 1 else "ies",
            ", truncated" if len(it) > cap else "")
        view += "\n".join("%s%s%s" % (r["name"], "/" if r["dir"] else "",
                                      "" if r["dir"] else
                                      "  %d bytes" % r["bytes"])
                          for r in rows)
        return ToolResult(text=_clip(view),
                          data={"path": str(p), "entries": rows,
                                "total_seen": len(it), "cap": cap})
    return list_dir


# --------------------------------------------------------------------------
# The manifest. DO NOT put a simulator name in any string below.
# --------------------------------------------------------------------------

SHELL_TOOL_DEFS = [
    ("run_shell",
     "Run a shell command and return its exit code, stdout and stderr. The "
     "working directory defaults to your working directory; pass `cwd` to run "
     "elsewhere. The command runs to completion (or is killed at the "
     "timeout); there is no interactive terminal, so never run anything that "
     "waits for input.",
     {"type": "object",
      "properties": {
          "command": {"type": "string",
                      "description": "the command line to execute"},
          "cwd": {"type": "string",
                  "description": "directory to run in (default: your working "
                                 "directory)"},
          "timeout_s": {"type": "number",
                        "description": "kill the command after this many "
                                       "seconds"},
      },
      "required": ["command"]}),
    ("read_file",
     "Read a text file and return its contents with line numbers. Use "
     "`start_line` and `max_lines` to page through a large file.",
     {"type": "object",
      "properties": {
          "path": {"type": "string",
                   "description": "file to read; relative paths resolve "
                                  "against your working directory"},
          "start_line": {"type": "integer",
                         "description": "1-based first line to return"},
          "max_lines": {"type": "integer",
                        "description": "how many lines to return"},
      },
      "required": ["path"]}),
    ("write_file",
     "Write text to a file, creating parent directories and overwriting any "
     "existing file. Writes are only permitted inside your working directory.",
     {"type": "object",
      "properties": {
          "path": {"type": "string",
                   "description": "file to write; relative paths resolve "
                                  "against your working directory"},
          "content": {"type": "string",
                      "description": "the full contents to write"},
      },
      "required": ["path", "content"]}),
    ("list_dir",
     "List the entries of a directory, with sizes. Pass `recursive` to walk "
     "the whole tree.",
     {"type": "object",
      "properties": {
          "path": {"type": "string",
                   "description": "directory to list (default: your working "
                                  "directory)"},
          "recursive": {"type": "boolean",
                        "description": "walk subdirectories too"},
          "max_entries": {"type": "integer",
                          "description": "cap on returned entries"},
      }}),
]

FACTORIES = {"run_shell": _make_run_shell, "read_file": _make_read_file,
             "write_file": _make_write_file, "list_dir": _make_list_dir}


def build(sandbox) -> ToolSet:
    specs = [ToolSpec(name=name, description=desc, input_schema=schema,
                      handler=FACTORIES[name](sandbox),
                      source="agentbench/shell/v1")
             for name, desc, schema in SHELL_TOOL_DEFS]
    return ToolSet(
        id="shell",
        description=("The cross-simulator baseline: a POSIX shell plus file "
                     "read/write/list. Byte-identical for every simulator; "
                     "nothing simulator-specific may appear in these "
                     "definitions (SPEC 4.1)."),
        specs=specs,
        env_policy=sandbox.env_policy(),
        notes=[
            "Tool DEFINITIONS are frozen across simulators; tool "
            "IMPLEMENTATIONS may differ (shell binary, container) and are "
            "deliberately excluded from tools_sha256.",
            "Working directory is the run's scratch dir. Writes are confined "
            "to it; reads of the benchmark's own package are refused.",
            "run_shell cannot be path-confined on a bare host: commands and "
            "results are scanned and any hit on the benchmark package is "
            "flagged leak_suspect in the trace. Full enforcement needs the "
            "per-run container from SPEC 4.3.",
            "The model sees at most %d characters of a result; the trace "
            "keeps the whole thing." % MODEL_TEXT_LIMIT,
        ])
