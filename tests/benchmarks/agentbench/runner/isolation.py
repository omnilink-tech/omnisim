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

"""Per-run isolation: scratch dir, ports, log path, env, path guard.

SPEC 4.3 wants one container per ``(task, sim, condition, repeat)``. On a bare
host we cannot have that, so this module implements the parts that *are*
achievable and is explicit about the parts that are not -- because an
isolation claim that overstates itself is worse than no claim.

Achieved here:

* **Scratch dir** -- the agent's working directory, provided by the harness,
  the only writable location the file tools permit.
* **Ports** -- a per-run ``(harness, supervisor)`` pair, probed free, from a
  range that does not collide with the grader's own probe range (the grader
  uses ``6900-7000``; the agent gets ``7100-7200``) or with a developer's
  hand-started harness on ``6789/6790``.
* **Log path** -- a per-run ``OMNISIM_LOG_PATH``. Without it parallel runs
  clobber the shared ``omnisim_log.txt`` and no run's engine log is readable
  after the fact.
* **Environment** -- built from an **allowlist**, not inherited. Notably
  absent: ``ANTHROPIC_API_KEY`` (the agent must not be able to spend the
  budget it is being measured against, or exfiltrate the key),
  ``AGENTBENCH_*`` (``AGENTBENCH_EXTERNAL_ARTIFACT`` would hand the agent a
  finished artifact), and ``PYTHONHASHSEED`` -- that last one is the subtle
  one: A1's ``husky_random`` seeds its RNG from ``hash(robot.getName())``, so
  pinning the hash seed silently changes the task's difficulty.
* **Path guard** -- the file tools refuse to read anything under
  ``tests/benchmarks/agentbench`` (the oracle solution, the graders and their
  thresholds all live there) and refuse to write anywhere but the scratch dir.
  *Only* the scratch dir: the run dir sits one level up and is where the
  engine resolves a scratch world's project root to, so a writable run dir let
  an agent drop a controller into ``<run_dir>/controllers/`` and shadow the
  grader's own recorder. Reads of the run dir are still allowed and still do
  not trip the leak detector (``own_roots``).
* **Read deny-list (the docs-ablation cell, SPEC 6.3 / plan Phase R item 6)**
  -- ``AGENTBENCH_READ_DENY`` names repo-relative paths (or the preset
  ``docs_ablation``) that the file tools treat as **nonexistent**: hidden from
  ``list_dir`` (entries and counts alike), and a ``read_file`` attempt returns
  the byte-identical error a missing path returns, so the agent cannot infer
  what is being hidden from the shape of the refusal. See
  ``PathGuard.is_hidden`` and ``parse_read_deny``.

**Not achieved, stated plainly:** ``run_shell`` executes a real shell, and a
shell cannot be confined by a path guard. An agent that goes looking can read
the grader. Full enforcement needs the container from SPEC 4.3. What we do
instead is *detect* it: every shell command and every tool result is scanned
for the guarded prefixes -- the benchmark package *and* any hidden roots --
and any hit is flagged ``leak_suspect`` in the trace, so a reviewer can grep
one field rather than read a transcript. A published run on a bare host must
carry that caveat. The deny-list shares the limit twice over: ``run_shell``
can still ``cat`` an ablated file (flagged by absolute-path mention, never
prevented), and an agent that reads its own run dir (legal, ``own_roots``)
can find the deny list's *names* -- not contents -- in the trace's
``runner_config`` event and the dumped tool-set manifest. A published
ablation row on a bare host must carry both caveats.
"""

from __future__ import annotations

import os
import posixpath
import re
import shutil
import socket
from dataclasses import dataclass, field
from pathlib import Path

from agentbench.common.paths import AGENTBENCH, REPO

# Ports. The grader's harness probe uses [6900, 7000); the default developer
# harness is 6789/6790. The agent gets its own range so a run cannot collide
# with either.
AGENT_PORT_LO = 7100
AGENT_PORT_HI = 7200

# Environment variables the agent's subprocesses may inherit. Everything else
# is dropped. Keep this list short and justified.
ENV_ALLOW = (
    # Windows needs these to start any process at all.
    "SYSTEMROOT", "SystemRoot", "COMSPEC", "ComSpec", "PATHEXT", "WINDIR",
    "windir", "NUMBER_OF_PROCESSORS", "PROCESSOR_ARCHITECTURE",
    "USERPROFILE", "HOMEDRIVE", "HOMEPATH", "APPDATA", "LOCALAPPDATA",
    # POSIX basics.
    "HOME", "USER", "LOGNAME", "LANG", "LC_ALL", "TERM", "SHELL", "TZ",
    # Needed to run anything at all.
    "PATH", "TMPDIR", "TEMP", "TMP",
    # X / headless rendering on Linux.
    "DISPLAY", "XAUTHORITY", "LD_LIBRARY_PATH",
)

# Never passed through, even if it appears in ENV_ALLOW's shadow. Prefix match.
ENV_DENY_PREFIXES = (
    "ANTHROPIC_", "OPENAI_", "OMNI_KEY", "OMNILINK_", "AGENTBENCH_",
    "PYTHONHASHSEED", "GH_TOKEN", "GITHUB_TOKEN", "AWS_", "GOOGLE_",
)

# -- the read deny-list (docs-ablation cell, SPEC 6.3) ----------------------
#
# ``AGENTBENCH_READ_DENY`` names repo-relative paths that the agent's file
# tools treat as NONEXISTENT: they do not appear in listings, and a read
# attempt returns the exact error a missing path returns. This is a different
# mechanism from ``deny_roots`` on purpose -- the benchmark package's refusal
# is deliberately loud and names itself, whereas a docs-ablation cell must not
# let the agent infer *what* is being hidden from the shape of the error.
#
# Preset contents and inclusion rationale: see runner/config.py, which is the
# documented front door for this knob.
READ_DENY_PRESETS = {
    "docs_ablation": ("AGENTS.md", "CLAUDE.md", "docs/developer"),
}


def parse_read_deny(raw) -> tuple:
    """Parse ``AGENTBENCH_READ_DENY`` into a tuple of repo-relative paths.

    Entries are separated by ``:`` or ``;``. Each entry is either a preset
    name from :data:`READ_DENY_PRESETS` (case/hyphen-insensitive) or a
    repo-relative path (either slash style). Empty input -> empty tuple.

    Malformed entries **raise** rather than being dropped: a silently ignored
    deny entry would run the wrong cell while attributing its rows to the
    ablation, which is exactly the bookkeeping failure the campaign forbids.
    Absolute paths are rejected (the list is repo-relative by contract -- and
    ``:`` is a separator, so a Windows drive path cannot survive the split
    anyway), as are entries that normalize to the repo root or escape it.
    """
    if not raw:
        return ()
    out = []
    for part in re.split(r"[;:]", str(raw)):
        part = part.strip()
        if not part:
            continue
        key = part.lower().replace("-", "_")
        if key in READ_DENY_PRESETS:
            for item in READ_DENY_PRESETS[key]:
                if item not in out:
                    out.append(item)
            continue
        cleaned = part.replace("\\", "/")
        if posixpath.isabs(cleaned):
            raise ValueError(
                "read-deny entries must be repo-relative, got %r" % part)
        norm = posixpath.normpath(cleaned).rstrip("/")
        if norm in (".", "") or norm == ".." or norm.startswith("../"):
            raise ValueError(
                "read-deny entry %r would hide the repo root or escape it"
                % part)
        if norm not in out:
            out.append(norm)
    return tuple(out)


def port_free(port, host="127.0.0.1", timeout=0.25) -> bool:
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return False
    except OSError:
        return True
    finally:
        s.close()


def pick_port_pair(start=AGENT_PORT_LO, end=AGENT_PORT_HI):
    for p in range(start, end, 2):
        if port_free(p) and port_free(p + 1):
            return p, p + 1
    raise RuntimeError("no free agent port pair in [%d, %d)" % (start, end))


def resolve_shell() -> tuple[list, str]:
    """A POSIX shell, and how we found it.

    SPEC 4.1 says the baseline is "a POSIX shell", so we look for ``bash``
    before falling back. On Windows the fallback is ``cmd.exe``, which is
    **not** POSIX -- that is recorded as a deviation rather than papered over,
    because a shell difference across simulators would break the very fairness
    property the ``shell`` condition exists to establish.
    """
    override = os.environ.get("AGENTBENCH_SHELL")
    if override and Path(override).exists():
        return [override, "-lc"], "override:%s" % override
    for cand in ("bash", "sh"):
        found = shutil.which(cand)
        if found:
            return [found, "-lc"], "which:%s" % found
    for cand in (r"C:\Program Files\Git\bin\bash.exe",
                 r"C:\msys64\usr\bin\bash.exe",
                 str(REPO / "msys64" / "usr" / "bin" / "bash.exe")):
        if Path(cand).exists():
            return [cand, "-lc"], "known-path:%s" % cand
    if os.name == "nt":
        return [os.environ.get("COMSPEC", "cmd.exe"), "/c"], "fallback:cmd.exe"
    return ["/bin/sh", "-c"], "fallback:/bin/sh"


class PathGuard:
    """Which paths the file tools may read and write.

    Deny beats allow. The deny list is the benchmark's own package: the oracle
    solution, the grader thresholds and the task metadata are all in there, and
    an agent that reads them is not solving the task.
    """

    def __init__(self, *, write_roots, read_roots=(), deny_roots=(),
                 own_roots=None, hide_roots=()):
        self.write_roots = [Path(p).resolve() for p in write_roots]
        self.read_roots = [Path(p).resolve() for p in read_roots]
        self.deny_roots = [Path(p).resolve() for p in deny_roots]
        # Hidden roots (the docs-ablation surface). Unlike deny_roots, whose
        # refusal is loud, anything under a hide root must be INDISTINGUISHABLE
        # from a nonexistent path -- see is_hidden.
        self.hide_roots = [Path(p).resolve() for p in hide_roots]
        # Paths that BELONG to this run (its scratch dir and its run dir).
        # Deliberately a superset of write_roots: the run dir must stay
        # readable and must not trip the leak detector, but it must NOT be
        # writable -- see Sandbox.create.
        self.own_roots = [Path(p).resolve()
                          for p in (own_roots if own_roots is not None
                                    else write_roots)]

    @staticmethod
    def _under(path: Path, root: Path) -> bool:
        try:
            path.relative_to(root)
            return True
        except ValueError:
            return False

    def resolve(self, path, base: Path) -> Path:
        p = Path(path)
        if not p.is_absolute():
            p = base / p
        # No strict=True: we must be able to name a file that does not exist
        # yet (write_file) while still normalizing ".." away.
        return Path(os.path.normpath(str(p)))

    def is_hidden(self, path) -> bool:
        """True when ``path`` is under a hidden (read-denied) root.

        THE PROPERTY THIS ENFORCES: a hidden path must be indistinguishable
        from a nonexistent one, so the agent cannot infer what is being hidden
        from the shape of the refusal. Callers therefore never surface this
        predicate in an error message -- they take the same "not a file" /
        "not a directory" branch a genuinely missing path takes, and listings
        simply omit hidden entries (from the count too).

        Both the normalized path and its symlink-resolved form are tested, so
        a symlink planted in the scratch dir cannot alias a hidden file back
        into view (``PathGuard.resolve`` normalizes ``..`` but deliberately
        does not resolve symlinks). Empty ``hide_roots`` short-circuits, so a
        run with no deny-list pays nothing and behaves identically.
        """
        if not self.hide_roots:
            return False
        p = Path(path)
        cands = [p]
        try:
            rp = p.resolve()
            if rp != p:
                cands.append(rp)
        except OSError:
            pass
        return any(self._under(c, h) for c in cands for h in self.hide_roots)

    def check_read(self, path: Path):
        for d in self.deny_roots:
            if self._under(path, d):
                # Except the run's own directories, which live *inside* the
                # denied tree when results are written under results/.
                if any(self._under(path, w) for w in self.own_roots):
                    return None
                return ("path is inside the benchmark's own package (%s) -- "
                        "it holds the grader and the reference solution and "
                        "is not readable by the agent" % d)
        if not self.read_roots:
            return None
        if any(self._under(path, r) for r in self.read_roots):
            return None
        return ("path is outside the readable roots (%s)"
                % ", ".join(str(r) for r in self.read_roots))

    def check_write(self, path: Path):
        if not any(self._under(path, w) for w in self.write_roots):
            return ("writes are confined to the run's scratch dir (%s)"
                    % ", ".join(str(w) for w in self.write_roots))
        return self.check_read(path)

    @staticmethod
    def _norm(text) -> str:
        """Separator- and escaping-insensitive form for substring matching.

        Collapsing runs of slashes matters: a path that has been through
        ``json.dumps`` carries doubled backslashes, and a naive
        ``replace("\\\\", "/")`` would turn ``O:\\omnisim`` into ``O://omnisim``
        and miss the match.
        """
        import re
        return re.sub(r"[\\/]+", "/", str(text)).lower()

    def flags(self, text) -> bool:
        """True when ``text`` mentions a denied root -- a leak *suspicion*.

        Used on shell commands and tool results, where we cannot enforce.

        The run's own writable roots are blanked out first, because a run
        directory normally lives *inside* the denied tree
        (``tests/benchmarks/agentbench/results/...``) -- without this, every
        single tool call that mentions its own scratch path flags, and a signal
        that fires on everything is not a signal. Measured on the first
        end-to-end run: 5 of 6 calls were false positives before this.
        """
        if not text:
            return False
        low = self._norm(text)
        for w in self.own_roots:
            low = low.replace(self._norm(w), "<run>")
        # Hidden roots are scanned too: in a docs-ablation run, a shell command
        # or result that mentions an ablated path's absolute form is exactly
        # the leak the file tools cannot stop (run_shell is unconfinable on a
        # bare host), and a reviewer must be able to grep for it.
        return any(self._norm(d) in low
                   for d in list(self.deny_roots) + list(self.hide_roots))


@dataclass
class Sandbox:
    """Everything per-run that a tool implementation needs."""

    run_dir: Path
    scratch_dir: Path
    repo: Path = REPO
    harness_port: int = 0
    supervisor_port: int = 0
    log_path: Path | None = None
    shell_argv: list = field(default_factory=list)
    shell_source: str = ""
    guard: PathGuard | None = None
    deviations: list = field(default_factory=list)
    # Repo-relative paths hidden from the agent's file tools (docs-ablation
    # cell, SPEC 6.3). Parsed form -- see parse_read_deny.
    read_deny: tuple = ()

    @classmethod
    def create(cls, run_dir, scratch_dir, *, repo=REPO, ports=True,
               read_deny=None):
        run_dir = Path(run_dir)
        scratch_dir = Path(scratch_dir)
        run_dir.mkdir(parents=True, exist_ok=True)
        scratch_dir.mkdir(parents=True, exist_ok=True)
        hp, sp = pick_port_pair() if ports else (0, 0)
        argv, source = resolve_shell()
        # The deny-list defaults from the RUNNER's environment (the agent's
        # own env strips AGENTBENCH_* regardless). run_agent() builds the
        # sandbox without per-call arguments, so the env var is the one path
        # by which a real run selects the ablation cell; RunnerConfig.from_env
        # reads the same variable, so the row's recorded value and the guard's
        # enforced value agree by construction.
        if read_deny is None:
            read_deny = parse_read_deny(
                os.environ.get("AGENTBENCH_READ_DENY", ""))
        elif isinstance(read_deny, str):
            read_deny = parse_read_deny(read_deny)
        else:
            read_deny = tuple(read_deny)
        sb = cls(run_dir=run_dir, scratch_dir=scratch_dir, repo=Path(repo),
                 harness_port=hp, supervisor_port=sp,
                 log_path=run_dir / "agent_engine.log",
                 shell_argv=argv, shell_source=source,
                 read_deny=read_deny)
        # WRITES ARE SCRATCH-ONLY. The run dir used to be writable too, and
        # that is how an agent could plant a fake grader: the engine resolves a
        # world's project root by walking up for a "worlds" directory and
        # falling back to the world's parent's parent (WbProject.cpp
        # projectPathFromWorldFile), so a world at <run_dir>/scratch/x.wbt
        # makes <run_dir> the project -- and WbRobot::updateControllerDir
        # searches <project>/controllers/<name>/ BEFORE the grader's
        # WEBOTS_EXTRA_PROJECT_PATH. One write_file to
        # ../controllers/agentbench_recorder/ shadowed the grader's own
        # recorder and let the agent author every "measured" number.
        # adapters/omnisim/headless.controller_shadow_check is the enforcing
        # half (run_shell cannot be path-confined); this is the other.
        sb.guard = PathGuard(write_roots=[scratch_dir],
                             read_roots=[repo, scratch_dir, run_dir],
                             own_roots=[scratch_dir, run_dir],
                             deny_roots=[AGENTBENCH],
                             hide_roots=[Path(repo) / r for r in read_deny])
        if source.startswith("fallback:cmd.exe"):
            sb.deviations.append(
                "no POSIX shell found; run_shell used cmd.exe, which is NOT "
                "the byte-identical baseline the shell condition assumes")
        (run_dir / "tool_results").mkdir(exist_ok=True)
        return sb

    # -- environment -----------------------------------------------------
    def env(self) -> dict:
        """The environment the agent's subprocesses get. Allowlist only."""
        out = {}
        for k, v in os.environ.items():
            if any(k.upper().startswith(p) for p in ENV_DENY_PREFIXES):
                continue
            if k in ENV_ALLOW:
                out[k] = v
        # The simulator is "installed in the image": these are inherent to the
        # cell, not a hint about the task.
        out["OMNISIM_HOME"] = str(self.repo)
        out["WEBOTS_HOME"] = str(self.repo)
        if self.log_path:
            out["OMNISIM_LOG_PATH"] = str(self.log_path)
        if self.harness_port:
            out["OMNISIM_HARNESS_PORT"] = str(self.harness_port)
            out["OMNISIM_SUPERVISOR_PORT"] = str(self.supervisor_port)
            out["OMNISIM_HARNESS_URL"] = self.harness_url
        return out

    @property
    def harness_url(self) -> str:
        return "http://127.0.0.1:%d" % self.harness_port

    def env_policy(self) -> dict:
        """Publishable description of the env policy, for the manifest."""
        pol = {"mode": "allowlist",
               "allow": sorted(ENV_ALLOW),
               "deny_prefixes": sorted(ENV_DENY_PREFIXES),
               "injected": ["OMNISIM_HOME", "WEBOTS_HOME",
                            "OMNISIM_LOG_PATH", "OMNISIM_HARNESS_PORT",
                            "OMNISIM_SUPERVISOR_PORT", "OMNISIM_HARNESS_URL"],
               "notes": [
                   "PYTHONHASHSEED is deliberately NOT set: husky_random "
                   "seeds from hash(name), so pinning it would change the "
                   "task's difficulty.",
                   "ANTHROPIC_* / AGENTBENCH_* are stripped: the agent must "
                   "not reach the model API or an external artifact.",
               ]}
        # Present ONLY when non-empty, for two reasons: (1) an empty deny-list
        # must leave the manifest byte-identical to the pre-ablation baseline
        # (and to the checked-in runner/manifests/*.json), so a baseline row's
        # manifest_sha256 is stable; (2) a NON-empty deny-list must change
        # manifest_sha256 -- env_policy is hashed into it -- so an ablation
        # row is mechanically attributable to its cell.
        if self.read_deny:
            pol["read_deny"] = {
                "mode": "hidden",
                "roots": list(self.read_deny),
                "note": "these repo paths are invisible to the file tools; a "
                        "read attempt returns the same error as a nonexistent "
                        "path (docs-ablation cell, SPEC 6.3)"}
        return pol

    def as_dict(self):
        return {"run_dir": str(self.run_dir),
                "scratch_dir": str(self.scratch_dir),
                "repo": str(self.repo),
                "harness_port": self.harness_port,
                "supervisor_port": self.supervisor_port,
                "log_path": str(self.log_path) if self.log_path else None,
                "shell": self.shell_argv, "shell_source": self.shell_source,
                "read_deny": list(self.read_deny),
                "deviations": list(self.deviations)}

    def variables(self) -> dict:
        """``{{VAR}}`` values available to a scripted replay."""
        from agentbench.common.paths import HUSKY_URDF, as_wbt_path
        return {
            "SCRATCH": as_wbt_path(self.scratch_dir),
            "RUN_DIR": as_wbt_path(self.run_dir),
            "REPO": as_wbt_path(self.repo),
            "HUSKY_URDF": as_wbt_path(HUSKY_URDF),
            "HARNESS_PORT": str(self.harness_port),
            "SUPERVISOR_PORT": str(self.supervisor_port),
            "HARNESS_URL": self.harness_url,
            "LOG_PATH": as_wbt_path(self.log_path) if self.log_path else "",
        }

    def teardown(self):
        """Assert the run's ports are free again (SPEC 4.3 teardown).

        Returns a list of survivors; a non-empty list should make the row
        ``INVALID`` -- something the agent started outlived the run.
        """
        survivors = []
        for name, port in (("harness", self.harness_port),
                           ("supervisor", self.supervisor_port)):
            if port and not port_free(port):
                survivors.append({"what": name, "port": port})
        return survivors
