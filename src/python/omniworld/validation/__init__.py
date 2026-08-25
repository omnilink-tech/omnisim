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

"""Validation harness.

``omniworld validate <world.omniworld>`` runs a set of checks against a
generated world and exits non-zero if any fail. Each check is a module
under ``validate/`` that exposes a callable returning a ``CheckResult``.

Four checks are in-process (cheap, always on): asset locality, prop
overlap, spawn reachability, viewpoint framing.

One check is headless-simulator (T1.7, opt-in via ``with_sim=True``
because it spawns a OmniSim process): load success + warning count.

Contact-pair budget and navigable-spawn-radius still need a custom
supervisor; they land as a follow-up.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .asset_locality import check_asset_locality
from .headless import HeadlessResult, discover_omnisim_bin, run_headless
from .overlap import check_prop_overlap
from .spawn import check_spawn


@dataclass(frozen=True)
class CheckResult:
    """Outcome of a single validator."""

    name: str
    passed: bool
    # Human-readable message; empty string on pass if the check has nothing
    # to say.
    detail: str = ""


@dataclass
class ValidationReport:
    """Aggregate outcome over every check."""

    world_path: Path
    results: list[CheckResult]

    @property
    def ok(self) -> bool:
        return all(r.passed for r in self.results)

    def format(self) -> str:
        """Human-readable multi-line summary."""
        lines: list[str] = []
        for r in self.results:
            marker = "PASS" if r.passed else "FAIL"
            suffix = f" -- {r.detail}" if r.detail else ""
            lines.append(f"  [{marker}] {r.name}{suffix}")
        lines.append("")
        lines.append("RESULT: " + ("ok" if self.ok else "FAILED"))
        return "\n".join(lines)


def _check_viewpoint(path: Path, text: str) -> "CheckResult":
    """Lazy wrapper -- see the note in ``__getattr__`` below."""
    from .viewpoint import check_viewpoint
    return check_viewpoint(path, text)


# The full set of in-process checks. sim-requiring checks (T1.7) will be
# added alongside these, gated by an ``--with-sim`` flag on the CLI.
_CHECKS: list[Callable[[Path, str], CheckResult]] = [
    check_asset_locality,
    check_prop_overlap,
    check_spawn,
    _check_viewpoint,
]


def __getattr__(name):
    # Lazy re-export. Importing .viewpoint eagerly here would make
    # ``python -m omniworld.validation.viewpoint`` (the audit / pre-push
    # entry point) double-import the module and warn.
    if name in ("ViewpointVerdict", "analyze_viewpoint", "check_viewpoint"):
        from . import viewpoint as _vp
        return getattr(_vp, name)
    raise AttributeError(name)


def validate(
    world_path: str | Path,
    *,
    with_sim: bool = False,
    headless_timeout_s: float = 15.0,
    headless_load_budget_s: float = 10.0,
    strict_warnings: bool = False,
) -> ValidationReport:
    """Run every in-process check on ``world_path``.

    When ``with_sim=True`` the headless-simulator check runs last.
    It takes several seconds per call; skip it on hot paths.
    """
    path = Path(world_path)
    text = path.read_text(encoding="utf-8")
    results = [check(path, text) for check in _CHECKS]
    if with_sim:
        results.append(_run_headless_check(
            path,
            timeout_s=headless_timeout_s,
            load_budget_s=headless_load_budget_s,
            strict_warnings=strict_warnings,
        ))
    return ValidationReport(world_path=path, results=results)


def _run_headless_check(
    world_path: Path,
    *,
    timeout_s: float,
    load_budget_s: float,
    strict_warnings: bool,
) -> CheckResult:
    try:
        result = run_headless(
            world_path,
            timeout_s=timeout_s,
            load_time_budget_s=load_budget_s,
            strict_warnings=strict_warnings,
        )
    except FileNotFoundError as exc:
        return CheckResult(
            name="headless_load",
            passed=False,
            detail=f"omnisim not available: {exc}",
        )
    return CheckResult(
        name="headless_load",
        passed=result.passed,
        detail=result.detail(),
    )


__all__ = [
    "CheckResult",
    "HeadlessResult",
    "ValidationReport",
    "ViewpointVerdict",
    "analyze_viewpoint",
    "check_viewpoint",
    "discover_omnisim_bin",
    "run_headless",
    "validate",
]
