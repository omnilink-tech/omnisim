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

"""Repo-wide pytest plumbing: the `engine` marker and the bring-up skip cap.

Two things, both about keeping a green run honest.

1. THE `engine` MARKER. `pytest -m "not engine"` is the engine-free unit lane
   (`make tests-unit`). No test file declares the marker by hand -- a module
   is marked `engine` here, at collection, when its SOURCE contains one of the
   launch-shaped patterns below (`ENGINE_PATTERNS`, plus the subprocess+omnisim
   pair). Derived from the ~60 engine-dependent files a 2026-09-02 audit found
   and deliberately biased towards MARKING: a pure test wrongly left out of the
   unit lane costs nothing, a launching test wrongly left in it spawns
   omnisim-bin on a box whose engine is being rebuilt. Known false positives
   (pure tests whose subprocess call is `git`, or the omniworld CLI) are
   accepted for that reason.

   `python tests/conftest.py --list-engine-free [DIR]` prints the collectable
   top-level test files of DIR that this logic leaves unmarked -- the Makefile
   reads the root-level unit files from it instead of a hand-written list.
   `--list-engine [DIR]` prints the marked ones with the reason.

2. THE BRING-UP SKIP CAP. Engine tests skip themselves when the engine does
   not come up ("bring-up flake", "did not come up", "Newton did not ...").
   Each skip is legitimate on its own; a run where the engine NEVER came up is
   not a pass, yet it reads green. This counts those skips and fails the
   session when there are more than OMNISIM_BRINGUP_SKIP_CAP (default 2; a
   negative value disables the cap). The terminal summary lists them.
"""

from __future__ import annotations

import functools
import os
import re
import sys
from pathlib import Path

import pytest

TESTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TESTS_DIR.parent

# --------------------------------------------------------------------------- #
# 1. engine marker                                                             #
# --------------------------------------------------------------------------- #

# Any ONE of these in a test module's source marks the module `engine`.
ENGINE_PATTERNS: tuple[tuple[str, re.Pattern[str]], ...] = (
    # The engine's own CLI flag: only a launch passes it.
    ("--batch flag", re.compile(r"--batch")),
    # omniworld.validation.run_headless / omnisim_dev run-headless as a CALL.
    ("run_headless() call", re.compile(r"\brun_headless\s*\(")),
    # The `skipif(_binary() is None)` idiom every Newton parity test uses.
    ("_binary() skipif", re.compile(r"_binary\(\)\s*(?:is|==)\s*None")),
    # A real resolve of the engine path (a monkeypatched name has no parens).
    ("resolve_omnisim_binary() call", re.compile(r"resolve_omnisim_binary\(\s*\)")),
    # OmniBench's engine launcher.
    ("engine_launch", re.compile(r"\bengine_launch\b")),
    # The CLI's engine-spawning env (omnisim.dev.runner / omnisim_env()).
    ("omnisim_env / dev.runner", re.compile(r"omnisim_env\(|omnisim\.dev\.runner")),
    # (No pattern for the harness / capture script PATH: pure unit tests read
    # `omnisim_harness.py` as source text and cite it in docstrings, while a
    # test that actually spawns it always trips the subprocess pair below.)
    # A live harness address.
    ("live harness URL", re.compile(r"OMNISIM_HARNESS_(?:INTEGRATION_)?URL")),
    # A test that skips itself when the engine did not come up launches one.
    ("bring-up skip", re.compile(r"did not come up|bring-up flake|Newton did not|"
                                 r"no built engine")),
)
# These two only count TOGETHER: a subprocess CALL plus any mention of the
# engine/runner. Pure unit tests import the same modules without calling out.
SUBPROCESS_CALL = re.compile(r"subprocess\.(?:run|Popen|check_output|check_call|call)\(")
ENGINE_TOKEN = re.compile(r"omnisim|headless_runner")

BRINGUP_SKIP_RE = re.compile(r"bring-up flake|did not come up|Newton did not", re.IGNORECASE)
BRINGUP_CAP_ENV = "OMNISIM_BRINGUP_SKIP_CAP"
BRINGUP_CAP_DEFAULT = 2


@functools.lru_cache(maxsize=None)
def engine_reasons(path: str) -> tuple[str, ...]:
    """Why a test module counts as `engine` -- () when it does not."""
    try:
        src = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ()
    reasons = [name for name, rx in ENGINE_PATTERNS if rx.search(src)]
    if SUBPROCESS_CALL.search(src) and ENGINE_TOKEN.search(src):
        reasons.append("subprocess call + omnisim")
    return tuple(reasons)


def is_engine_module(path: str | os.PathLike[str]) -> bool:
    return bool(engine_reasons(str(path)))


def pytest_configure(config: pytest.Config) -> None:
    # pytest.ini declares it too; this keeps `pytest tests/x` honest when run
    # from a directory that does not pick the ini up.
    config.addinivalue_line("markers", "engine: launches or requires the simulator binary")


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:  # noqa: ARG001
    for item in items:
        path = str(getattr(item, "path", None) or item.fspath)
        reasons = engine_reasons(path)
        if reasons:
            item.add_marker(pytest.mark.engine(reason=", ".join(reasons)))


# --------------------------------------------------------------------------- #
# 2. bring-up skip cap                                                         #
# --------------------------------------------------------------------------- #

def bringup_cap() -> int:
    raw = os.environ.get(BRINGUP_CAP_ENV, "")
    try:
        return int(raw) if raw.strip() else BRINGUP_CAP_DEFAULT
    except ValueError:
        return BRINGUP_CAP_DEFAULT


def skip_reason(report: pytest.TestReport) -> str:
    longrepr = getattr(report, "longrepr", None)
    if isinstance(longrepr, tuple) and len(longrepr) == 3:
        return str(longrepr[2])
    return str(longrepr or "")


def is_bringup_skip(report: pytest.TestReport) -> bool:
    return bool(report.skipped and BRINGUP_SKIP_RE.search(skip_reason(report)))


_BRINGUP_SKIPS: list[tuple[str, str]] = []


def pytest_runtest_logreport(report: pytest.TestReport) -> None:
    if is_bringup_skip(report):
        _BRINGUP_SKIPS.append((report.nodeid, skip_reason(report)))


def pytest_terminal_summary(terminalreporter, exitstatus, config) -> None:  # noqa: ARG001
    if not _BRINGUP_SKIPS:
        return
    cap = bringup_cap()
    over = cap >= 0 and len(_BRINGUP_SKIPS) > cap
    terminalreporter.section("engine bring-up skips", sep="=")
    for nodeid, reason in _BRINGUP_SKIPS:
        terminalreporter.line(f"  {nodeid}: {reason}")
    if over:
        terminalreporter.line(
            f"FAIL: {len(_BRINGUP_SKIPS)} engine bring-up skips exceed {BRINGUP_CAP_ENV}={cap}: "
            "a suite where the engine never comes up must not read green. Run "
            "`python -m omnisim doctor`, then one engine test by hand.")
    else:
        terminalreporter.line(f"{len(_BRINGUP_SKIPS)} engine bring-up skip(s), cap {cap}.")


@pytest.hookimpl(trylast=True)
def pytest_sessionfinish(session: pytest.Session, exitstatus) -> None:  # noqa: ARG001
    cap = bringup_cap()
    if cap >= 0 and len(_BRINGUP_SKIPS) > cap and session.exitstatus == 0:
        session.exitstatus = int(pytest.ExitCode.TESTS_FAILED)


# --------------------------------------------------------------------------- #
# CLI: the Makefile reads the engine-free root-level files from here           #
# --------------------------------------------------------------------------- #

def collectable_test_files(directory: Path) -> list[Path]:
    """Top-level `test_*.py` files of `directory` that a bare pytest would collect
    (pytest.ini ignores test_suite.py / test_sources.py / test_worlds.py)."""
    ignored = {"test_suite.py", "test_sources.py", "test_worlds.py"}
    return sorted(p for p in directory.glob("test_*.py") if p.name not in ignored)


def _main(argv: list[str]) -> int:
    # LF only: the Makefile splices this output into a pytest command line, and
    # a Windows CR LF would leave a CR glued to every filename.
    try:
        sys.stdout.reconfigure(newline="\n")
    except (AttributeError, ValueError):
        pass
    if not argv or argv[0] not in ("--list-engine-free", "--list-engine"):
        print("usage: conftest.py --list-engine-free [DIR]  |  --list-engine [DIR]", file=sys.stderr)
        return 2
    directory = Path(argv[1]) if len(argv) > 1 else TESTS_DIR
    want_engine = argv[0] == "--list-engine"
    for path in collectable_test_files(directory):
        reasons = engine_reasons(str(path))
        if bool(reasons) == want_engine:
            rel = path.resolve().relative_to(REPO_ROOT).as_posix()
            print(f"{rel}  # {', '.join(reasons)}" if want_engine else rel)
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
