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

# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Per-run usage snapshots into ``omnilink-reports/usage/<agent>/<iso>.json``.

Public API:

    record = UsageRecord(agent="husky_maze", headless=False)
    record.start()
    # ... run a thing ...
    record.finish(exit_code=0)

Or, the one-shot helper for callers that already know the times:

    write_run_snapshot(
        agent="husky_maze",
        started_at=t0,
        ended_at=t1,
        exit_code=0,
        headless=False,
        duration_capped=None,
    )

This is intentionally cheap and dependency-free in the common case
where the OmniLink Python library isn't reachable — it falls back to
a no-usage record that still preserves the wall-clock window and
exit code.

Set ``OMNISIM_USAGE_SNAPSHOT=0`` to skip the snapshot for one run.
"""

from __future__ import annotations

import datetime as _dt
import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional


_THIS_FILE = Path(__file__).resolve()
_REPO_ROOT = _THIS_FILE.parents[3]  # _lib -> production -> agents -> repo
USAGE_ROOT = _REPO_ROOT / "omnilink-reports" / "usage"


def _utc_now() -> _dt.datetime:
    return _dt.datetime.now(_dt.timezone.utc).replace(microsecond=0)


def _filename_for(timestamp: _dt.datetime) -> str:
    """Filesystem-safe ISO-ish: '2026-05-13T12-44-01Z.json'."""
    return timestamp.strftime("%Y-%m-%dT%H-%M-%SZ.json")


def _iso(timestamp: _dt.datetime) -> str:
    return timestamp.strftime("%Y-%m-%dT%H:%M:%SZ")


def _snapshot_enabled() -> bool:
    return os.environ.get("OMNISIM_USAGE_SNAPSHOT", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )


@dataclass
class UsageRecord:
    """Wall-clock window + optional OmniLink usage delta around one run."""

    agent: str
    headless: bool = False
    duration_capped: Optional[int] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    _started_at: Optional[_dt.datetime] = field(default=None, init=False, repr=False)
    _baseline: Any = field(default=None, init=False, repr=False)
    _meter: Any = field(default=None, init=False, repr=False)
    _client: Any = field(default=None, init=False, repr=False)

    def start(self) -> _dt.datetime:
        """Read baseline usage rollup (best-effort) and record start time."""
        self._started_at = _utc_now()
        if not _snapshot_enabled():
            return self._started_at
        try:
            self._meter, self._client = _build_meter()
        except Exception:
            self._meter = None
            self._client = None
        if self._meter is not None:
            try:
                self._baseline = self._meter.start()
            except Exception:
                self._baseline = None
        return self._started_at

    def finish(self, *, exit_code: int = 0) -> Optional[Path]:
        """Read final usage rollup, build the snapshot, write JSON.

        Returns the snapshot's path on success, or ``None`` when
        snapshotting is disabled or the agent has no start timestamp.
        """
        if self._started_at is None:
            return None
        if not _snapshot_enabled():
            return None
        ended_at = _utc_now()
        wall_s = round((ended_at - self._started_at).total_seconds(), 1)

        usage_block: Dict[str, Any] = {"available": False}
        if self._meter is not None and self._baseline is not None:
            try:
                delta = self._meter.snapshot(baseline=self._baseline)
                usage_block = delta.to_dict()
                usage_block["available"] = True
            except Exception as exc:
                usage_block = {
                    "available": False,
                    "reason": f"snapshot failed: {exc.__class__.__name__}: {exc}",
                }
        elif not os.environ.get("OMNI_KEY", "").strip():
            usage_block["reason"] = "OMNI_KEY not set"
        else:
            usage_block["reason"] = "omnilink-lib unreachable"

        payload = {
            "agent": self.agent,
            "started_at": _iso(self._started_at),
            "ended_at": _iso(ended_at),
            "wall_s": wall_s,
            "exit_code": int(exit_code),
            "headless": bool(self.headless),
            "duration_capped": self.duration_capped,
            "usage": usage_block,
        }
        if self.extra:
            payload["extra"] = dict(self.extra)

        target_dir = USAGE_ROOT / self.agent
        try:
            target_dir.mkdir(parents=True, exist_ok=True)
            out_path = target_dir / _filename_for(ended_at)
            out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            return out_path
        except OSError as exc:
            # Writing the snapshot is best-effort. A read-only filesystem
            # or denied-path is not worth crashing the runner for.
            print(f"  [warn] could not write usage snapshot: {exc}")
            return None


def _build_meter():
    """Construct an OmniLink ``UsageMeter`` + client. Raises on failure.

    Imports are deferred so callers without omnilink-lib still get a
    useful UsageRecord that records the wall window + exit code.
    """
    from .runner_base import locate_omnilink_lib

    _inject_system_trust_store()
    locate_omnilink_lib()
    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        raise RuntimeError("OMNI_KEY not set")
    from omnilink.client import OmniLinkClient  # type: ignore[import-not-found]
    from omnilink.usage_meter import UsageMeter  # type: ignore[import-not-found]

    client = OmniLinkClient(omni_key=omni_key, base_url="https://www.omnilink-agents.com")
    return UsageMeter(client), client


_TRUSTSTORE_INJECTED = False


def _inject_system_trust_store() -> None:
    """Use the OS trust store for TLS when ``truststore`` is available.

    Some Windows boxes run TLS-intercepting antivirus (AVG, ESET, etc.)
    whose CA is installed in the Windows cert store but not in
    certifi's bundle. Without this, every ``requests`` call to
    omnilink-agents.com fails with SSL_CERT_VERIFICATION_FAILED.
    Safe no-op when truststore isn't installed.
    """
    global _TRUSTSTORE_INJECTED
    if _TRUSTSTORE_INJECTED:
        return
    try:
        import truststore  # type: ignore[import-not-found]
        truststore.inject_into_ssl()
        _TRUSTSTORE_INJECTED = True
    except Exception:
        # No truststore (or already-injected) — fine, requests will fall
        # back to certifi. The caller will get a clear SSL error if the
        # platform's cert chain isn't publicly rooted.
        pass


def write_run_snapshot(
    *,
    agent: str,
    started_at: _dt.datetime,
    ended_at: _dt.datetime,
    exit_code: int,
    headless: bool = False,
    duration_capped: Optional[int] = None,
    usage: Optional[Dict[str, Any]] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Optional[Path]:
    """Write a snapshot when the caller already has the times in hand.

    Mainly used by ``omnisim_run_agent.py`` which owns the wall window.
    When ``usage`` is None, the function only records the wall window
    and exit code (no live rollup query).
    """
    if not _snapshot_enabled():
        return None
    wall_s = round((ended_at - started_at).total_seconds(), 1)
    payload = {
        "agent": agent,
        "started_at": _iso(started_at),
        "ended_at": _iso(ended_at),
        "wall_s": wall_s,
        "exit_code": int(exit_code),
        "headless": bool(headless),
        "duration_capped": duration_capped,
        "usage": usage or {"available": False, "reason": "not collected"},
    }
    if extra:
        payload["extra"] = dict(extra)
    target_dir = USAGE_ROOT / agent
    try:
        target_dir.mkdir(parents=True, exist_ok=True)
        out_path = target_dir / _filename_for(ended_at)
        out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return out_path
    except OSError as exc:
        print(f"  [warn] could not write usage snapshot: {exc}")
        return None
