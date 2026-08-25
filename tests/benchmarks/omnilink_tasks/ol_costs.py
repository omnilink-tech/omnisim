# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Measured cost, from ``GET /api/agent-costs?agentName=<name>``.

1 credit = 1 USD (``api/agent-costs.ts``). The sampler takes a snapshot
before and after a task (and before/after a whole engine's suite) and records
the delta. Nothing is estimated: if a snapshot could not be taken, the cost is
``None`` and the row says why.

FOUR WAYS THIS CAN FAIL, AND WHAT EACH RECORDS
----------------------------------------------
* **503 MIGRATION_HINT** — the agent-attribution migration is not applied on
  the deployment being measured. ``credits_usd: None``,
  ``source: "endpoint 503 MIGRATION_HINT"``. Never 0.
* **401 / 403** — no or bad Omni Key. ``credits_usd: None``.
* **credits column not written at log time** — a known platform defect
  (``agents/production/_lib/cost.py`` documents ``credits_24h`` reading 0
  against 1.47M input units). If the credits delta is 0 while the unit delta
  is positive, the platform figure is recorded as ``None`` with
  ``source: "platform credits not written"``, and a clearly-labelled
  ``derived_usd`` is computed locally from the measured unit deltas and the
  price table in ``_lib/cost.py``. A derived figure is arithmetic over
  measured units, not an estimate of usage — but it is only as right as the
  price table, so it is never presented as the measured cost.
* **concurrent traffic on the same agent name** — the endpoint attributes by
  ``agent_name``, so anything else driving ``HuskySwarm`` on this key during
  the window lands in the delta. The sampler records the request-count delta
  next to the credits so an implausible attribution is visible.
"""

from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

DEFAULT_BASE_URL = "https://www.omnilink-agents.com"
REPO_ROOT = Path(__file__).resolve().parents[3]

# Give the platform a moment to write the usage row for the last chat turn
# before sampling "after". Measured behaviour is write-on-completion, but the
# row lands asynchronously.
SETTLE_S = 3.0


@dataclass
class CostSnapshot:
    ok: bool
    credits: Optional[float] = None
    input_units: Optional[float] = None
    output_units: Optional[float] = None
    requests: Optional[int] = None
    live_credits_per_minute: Optional[float] = None
    sampled_at: str = ""
    error: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {"ok": self.ok, "credits": self.credits,
                "input_units": self.input_units,
                "output_units": self.output_units,
                "requests": self.requests,
                "sampled_at": self.sampled_at,
                "error": self.error or None}


class AgentCostSampler:
    """Snapshot / delta over ``/api/agent-costs``.

    ``fetch`` is injectable so the offline harness test can drive the 503 and
    credits-not-written paths without a network or an account.
    """

    def __init__(self, *, agent_name: str, omni_key: str,
                 base_url: str = DEFAULT_BASE_URL,
                 timeout_s: float = 30.0,
                 fetch: Optional[Any] = None) -> None:
        self.agent_name = agent_name
        self.omni_key = omni_key
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s
        self._fetch = fetch or self._http_fetch

    # -- transport -----------------------------------------------------

    def _http_fetch(self) -> Dict[str, Any]:
        url = (f"{self.base_url}/api/agent-costs?"
               + urllib.parse.urlencode({"agentName": self.agent_name}))
        req = urllib.request.Request(
            url, headers={"Authorization": f"Bearer {self.omni_key}",
                          "Accept": "application/json"}, method="GET")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as r:
                return {"status": r.status,
                        "body": json.loads(r.read().decode("utf-8") or "{}")}
        except urllib.error.HTTPError as exc:
            raw = exc.read().decode("utf-8", "replace")
            try:
                body = json.loads(raw or "{}")
            except Exception:
                body = {"raw": raw[:300]}
            return {"status": exc.code, "body": body}
        except Exception as exc:  # noqa: BLE001
            return {"status": 0, "body": {"error": f"{type(exc).__name__}: {exc}"}}

    # -- snapshots -----------------------------------------------------

    def snapshot(self) -> CostSnapshot:
        if not self.omni_key:
            return CostSnapshot(False, error="no OMNI_KEY")
        try:
            resp = self._fetch()
        except Exception as exc:  # noqa: BLE001
            return CostSnapshot(False, error=f"{type(exc).__name__}: {exc}")
        status = int(resp.get("status") or 0)
        body = resp.get("body") or {}
        if status == 503 and body.get("code") == "MIGRATION_HINT":
            return CostSnapshot(False, error="endpoint 503 MIGRATION_HINT")
        if status != 200 or not body.get("ok"):
            err = body.get("error") or body.get("code") or f"HTTP {status}"
            return CostSnapshot(False, error=str(err)[:200])
        totals = body.get("totals") or {}
        live = body.get("live") or {}
        return CostSnapshot(
            True,
            credits=_num(totals.get("credits")),
            input_units=_num(totals.get("inputUnits")),
            output_units=_num(totals.get("outputUnits")),
            requests=int(_num(totals.get("requests")) or 0),
            live_credits_per_minute=_num(live.get("creditsPerMinute")),
            sampled_at=str(live.get("sampledAt") or ""),
        )

    def delta(self, before: CostSnapshot, after: CostSnapshot, *,
              engine: Optional[str] = None) -> Dict[str, Any]:
        """Cost of the interval between two snapshots.

        Returns a dict that ALWAYS carries ``credits_usd`` — set to ``None``
        whenever the figure was not measured. Read the ``source`` field before
        quoting the number.
        """
        out: Dict[str, Any] = {
            "credits_usd": None,
            "derived_usd": None,
            "input_units_delta": None,
            "output_units_delta": None,
            "requests_delta": None,
            "engine": engine,
            "source": "not sampled",
            "notes": [],
        }
        if not before.ok or not after.ok:
            out["source"] = (f"unmeasured: before={before.error or 'ok'} "
                             f"after={after.error or 'ok'}").strip()
            return out
        d_in = _sub(after.input_units, before.input_units)
        d_out = _sub(after.output_units, before.output_units)
        d_req = _sub(after.requests, before.requests)
        d_cred = _sub(after.credits, before.credits)
        out["input_units_delta"] = d_in
        out["output_units_delta"] = d_out
        out["requests_delta"] = None if d_req is None else int(d_req)

        if d_cred is not None and d_cred > 0:
            out["credits_usd"] = round(d_cred, 6)
            out["source"] = "GET /api/agent-costs totals.credits delta"
        elif d_cred == 0 and (d_in or 0) + (d_out or 0) > 0:
            out["source"] = "platform credits not written (units moved, credits did not)"
            out["notes"].append(
                "The platform logged usage units for this window but recorded "
                "0 credits. That is the known write-time defect documented in "
                "agents/production/_lib/cost.py; the measured cost is "
                "unavailable, not zero.")
        elif d_cred == 0:
            out["credits_usd"] = 0.0
            out["source"] = ("GET /api/agent-costs totals.credits delta "
                             "(no usage recorded in the window)")
        else:
            out["source"] = "unmeasured: credits totals missing from response"

        derived = _derive_usd(engine, d_in, d_out)
        if derived is not None:
            out["derived_usd"] = round(derived, 6)
            out["derived_source"] = ("locally priced from the MEASURED unit "
                                     "deltas using PRICES in "
                                     "agents/production/_lib/cost.py — "
                                     "arithmetic, not the platform's figure")
        if d_req == 0 and (d_in or 0) == 0:
            out["notes"].append("Zero requests attributed to this agent name "
                                "in the window — check the agent is sending "
                                "agentName, or that the run actually reached "
                                "the platform.")
        return out


def _num(v: Any) -> Optional[float]:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f


def _sub(a: Optional[float], b: Optional[float]) -> Optional[float]:
    if a is None or b is None:
        return None
    return a - b


def _derive_usd(engine: Optional[str], d_in: Optional[float],
                d_out: Optional[float]) -> Optional[float]:
    """Price measured unit deltas with the repo's own table. Labelled derived."""
    if not engine or d_in is None or d_out is None:
        return None
    try:
        sys.path.insert(0, str(REPO_ROOT / "agents" / "production"))
        from _lib.cost import cost_usd, kind_for_engine  # type: ignore
    except Exception:
        return None
    try:
        # An engine the price table does not know prices at 0.0, which would
        # print as "$0.0000" and read as "this was free". Absence, not zero.
        if kind_for_engine(engine) is None:
            return None
        return float(cost_usd(engine, d_in, d_out))
    except Exception:
        return None


class NullCostSampler:
    """Used when no Omni Key is available (the `local` control arm)."""

    def __init__(self, reason: str = "no credential; cost not applicable") -> None:
        self.reason = reason

    def snapshot(self) -> CostSnapshot:
        return CostSnapshot(False, error=self.reason)

    def delta(self, before: CostSnapshot, after: CostSnapshot, *,
              engine: Optional[str] = None) -> Dict[str, Any]:
        return {"credits_usd": None, "derived_usd": None, "engine": engine,
                "input_units_delta": None, "output_units_delta": None,
                "requests_delta": None, "source": self.reason, "notes": []}


def settle() -> None:
    time.sleep(SETTLE_S)
