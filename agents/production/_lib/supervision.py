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
# Licensed under the Apache License, Version 2.0.

"""Model-independent contracts for supervised agent execution.

The language model decides *what* to do.  These helpers make the runtime
responsible for the parts that should never depend on prose interpretation:

* normalize heterogeneous tool results into one outcome vocabulary;
* execute dependent plan steps in order and stop at the first unverified step;
* reject parallel work that claims the same exclusive resource twice.

The module intentionally has no OmniLink or OmniSim dependency so every
production agent can share it and its behavior can be unit tested cheaply.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence


_COMPLETION_KEYS = (
    "completed",
    "success",
    "arrived",
    "within_tolerance",
    "all_completed",
    "mission_complete",
    "captain_mission_complete",
)
_VERIFICATION_KEYS = (
    "verified",
    "within_tolerance",
    "arrived",
    "mission_complete",
    "captain_mission_complete",
)


def failure_error(result: Any) -> Optional[Any]:
    """Return the value of ``error`` only when it actually marks a FAILURE.

    A shipped bridge names its numeric residual ``error``: the maze bridge's
    blocking ``wait: true`` payload returns
    ``{"commanded": 1.0, "achieved": 0.97, "error": -0.03, "unit": "m"}``
    for a healthy, settled drive.  Treating that as a failure marked every
    successful blocking drive/turn as failed -- only a residual of exactly
    0.0 escaped -- so the runner also answered ``status: "err"`` over HTTP
    for work that went right.

    Rule: a NUMBER in ``error`` is a residual, never a failure.  Strings,
    dicts, lists and ``True`` are failures.  ``False``/``None``/empty are not.

    Follow-up (not ours to make -- the bridge lives outside this package):
    ``husky_omnilink_bridge`` should rename that field to ``residual`` so the
    ambiguity disappears at the source rather than being decoded here.
    """

    if not isinstance(result, Mapping):
        return None
    error = result.get("error")
    if isinstance(error, bool):
        return error or None
    if isinstance(error, (int, float)):
        return None
    return error or None


@dataclass(frozen=True)
class ActionOutcome:
    """Portable outcome attached to activity records and mission ledgers."""

    state: str
    accepted: Optional[bool]
    completed: Optional[bool]
    verified: Optional[bool]
    failed: bool
    reason: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def normalize_outcome(result: Any) -> ActionOutcome:
    """Map a shipped tool's result onto a conservative common contract.

    Unknown is deliberately distinct from success.  A response such as
    ``{"accepted": true}`` proves only that work started; it does not prove
    that a robot reached its postcondition.
    """

    if not isinstance(result, Mapping):
        return ActionOutcome(
            state="unknown",
            accepted=None,
            completed=None,
            verified=None,
            failed=False,
            reason="result is not a structured object",
        )

    error = failure_error(result)
    accepted = result.get("accepted") if isinstance(result.get("accepted"), bool) else None
    completed = next(
        (result[key] for key in _COMPLETION_KEYS if isinstance(result.get(key), bool)),
        None,
    )
    verified = next(
        (result[key] for key in _VERIFICATION_KEYS if isinstance(result.get(key), bool)),
        None,
    )
    if verified is None and completed is True:
        # A tool-owned ``completed`` postcondition is verification.  In
        # contrast, ``accepted`` alone remains only accepted.
        verified = True

    explicit_failure = any(
        isinstance(result.get(key), bool) and result.get(key) is False
        for key in _COMPLETION_KEYS
    )
    failed = bool(error) or explicit_failure
    if failed:
        state = "failed"
    elif verified is True or completed is True:
        state = "completed"
    elif accepted is True:
        state = "accepted"
    else:
        state = "unknown"

    reason: Optional[str] = None
    if error:
        reason = str(error)
    elif explicit_failure:
        reason = str(result.get("reason") or result.get("hint") or "postcondition was not met")

    return ActionOutcome(
        state=state,
        accepted=accepted,
        completed=completed,
        verified=verified,
        failed=failed,
        reason=reason,
    )


Verifier = Callable[[Mapping[str, Any], Any], bool]
Executor = Callable[[Mapping[str, Any]], Any]


def execute_verified_sequence(
    steps: Sequence[Mapping[str, Any]],
    executor: Executor,
    *,
    verifier: Optional[Verifier] = None,
    stop_on_failure: bool = True,
) -> Dict[str, Any]:
    """Execute dependent steps with an auditable, fail-closed ledger."""

    started = time.monotonic()
    entries: List[Dict[str, Any]] = []
    failed_step: Optional[str] = None

    for index, step in enumerate(steps):
        step_id = str(step.get("id") or f"step_{index + 1}")
        step_started = time.monotonic()
        try:
            result = executor(step)
        except Exception as exc:  # runtime boundary: preserve a ledger on crash
            result = {
                "error": "step_execution_crashed",
                "detail": f"{exc.__class__.__name__}: {exc}",
            }
        outcome = normalize_outcome(result)
        if verifier is None:
            verified = outcome.verified is True and not outcome.failed
        else:
            try:
                verified = bool(verifier(step, result))
            except Exception as exc:
                verified = False
                if isinstance(result, dict):
                    result = dict(result)
                    result.setdefault(
                        "verification_error",
                        f"{exc.__class__.__name__}: {exc}",
                    )
        entry = {
            "index": index,
            "id": step_id,
            "verified": verified,
            "outcome": outcome.to_dict(),
            "elapsed_s": round(time.monotonic() - step_started, 3),
            "result": result,
        }
        entries.append(entry)
        if not verified:
            failed_step = step_id
            if stop_on_failure:
                break

    completed = failed_step is None and len(entries) == len(steps)
    return {
        "completed": completed,
        "verified": completed,
        "requested_steps": len(steps),
        "completed_steps": sum(1 for entry in entries if entry["verified"]),
        "failed_step": failed_step,
        "stopped_early": len(entries) < len(steps),
        "elapsed_s": round(time.monotonic() - started, 3),
        "steps": entries,
    }


def find_duplicate_resource_claims(
    items: Iterable[Mapping[str, Any]],
    resource_of: Callable[[Mapping[str, Any]], Optional[str]],
) -> List[Dict[str, Any]]:
    """Return every duplicate exclusive-resource claim in stable order."""

    owners: Dict[str, int] = {}
    duplicates: List[Dict[str, Any]] = []
    for index, item in enumerate(items):
        resource = resource_of(item)
        if not resource:
            continue
        if resource in owners:
            duplicates.append(
                {
                    "resource": resource,
                    "first_index": owners[resource],
                    "duplicate_index": index,
                }
            )
        else:
            owners[resource] = index
    return duplicates
