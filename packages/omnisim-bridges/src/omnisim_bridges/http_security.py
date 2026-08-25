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

"""Shared HTTP safety helpers for OmniLink robot-control surfaces.

The bridge is intentionally usable from curl and local agent processes (which
usually send no ``Origin`` header), while browser requests are restricted to
explicitly trusted origins.  A non-loopback bind is refused unless a bearer
token is configured: exposing unauthenticated motion verbs to a LAN is never a
safe default.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Optional, Set
from urllib.parse import urlsplit


DEFAULT_MAX_REQUEST_BYTES = 4 * 1024 * 1024
WIRE_VERSION = "1.0"
WIRE_SERVICE = "robot_bridge"
_LOOPBACK_HOSTS = {"127.0.0.1", "::1", "localhost"}


@dataclass(frozen=True)
class RequestError(Exception):
    status: int
    code: str
    message: str
    details: Optional[Dict[str, Any]] = None


def error_envelope(code: str, message: str, details: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    return {
        "ok": False,
        "error": code,
        "message": message,
        "details": details or {},
    }


def configured_token(explicit: Optional[str] = None) -> str:
    return (explicit if explicit is not None else os.environ.get("OMNISIM_BRIDGE_TOKEN", "")).strip()


def validate_bind(host: str, token: str) -> None:
    if host.strip().lower() not in _LOOPBACK_HOSTS and not token:
        raise ValueError(
            "A non-loopback robot bridge requires OMNISIM_BRIDGE_TOKEN (or token=...). "
            "Use host='127.0.0.1' for local-only control."
        )


def _origin_from_url(value: str) -> str:
    parsed = urlsplit(value.strip())
    if parsed.scheme and parsed.netloc:
        return f"{parsed.scheme.lower()}://{parsed.netloc.lower()}"
    return value.strip().rstrip("/").lower()


def allowed_origins(extra: Optional[Iterable[str]] = None) -> Set[str]:
    values = {
        "https://www.omnilink-agents.com",
        "https://omnilink-agents.com",
        "http://127.0.0.1",
        "http://localhost",
    }
    base = os.environ.get("OMNILINK_BASE_URL", "").strip()
    if base:
        values.add(_origin_from_url(base))
    configured = os.environ.get("OMNISIM_ALLOWED_ORIGINS", "")
    values.update(v.strip() for v in configured.split(",") if v.strip())
    if extra:
        values.update(extra)
    return {_origin_from_url(v) for v in values if v}


def checked_origin(headers: Mapping[str, str], trusted: Set[str]) -> Optional[str]:
    origin = (headers.get("Origin") or "").strip()
    if not origin:
        return None  # curl, SDKs, and same-process clients
    normalized = _origin_from_url(origin)
    if normalized not in trusted:
        raise RequestError(403, "origin_not_allowed", "Browser origin is not allowed.", {"origin": origin})
    return origin


def check_authorization(headers: Mapping[str, str], token: str) -> None:
    if not token:
        return
    auth = (headers.get("Authorization") or "").strip()
    alternate = (headers.get("X-OmniSim-Token") or "").strip()
    supplied = auth[7:].strip() if auth.lower().startswith("bearer ") else alternate
    if supplied != token:
        raise RequestError(401, "unauthorized", "A valid robot-bridge token is required.")


def check_protocol_version(headers: Mapping[str, str]) -> None:
    requested = (headers.get("Accept-Protocol-Version") or "").strip()
    if requested and requested != WIRE_VERSION:
        raise RequestError(
            409,
            "protocol_unsupported",
            f"This robot bridge supports OmniSim Wire {WIRE_VERSION}.",
            {"requested": requested, "supported": [WIRE_VERSION]},
        )


def max_request_bytes() -> int:
    raw = os.environ.get("OMNISIM_MAX_REQUEST_BYTES", str(DEFAULT_MAX_REQUEST_BYTES))
    try:
        value = int(raw)
    except ValueError:
        return DEFAULT_MAX_REQUEST_BYTES
    return max(1024, min(value, 64 * 1024 * 1024))


def _reject_nonfinite_json(value: str) -> None:
    raise ValueError(f"non-finite JSON constant: {value}")


def read_json(handler: Any, *, allow_empty: bool = True) -> Dict[str, Any]:
    raw_length = handler.headers.get("Content-Length", "0")
    try:
        length = int(raw_length)
    except (TypeError, ValueError) as exc:
        raise RequestError(400, "invalid_content_length", "Content-Length must be an integer.") from exc
    if length < 0:
        raise RequestError(400, "invalid_content_length", "Content-Length cannot be negative.")
    if length == 0:
        if allow_empty:
            return {}
        raise RequestError(400, "missing_body", "A JSON request body is required.")
    limit = max_request_bytes()
    if length > limit:
        raise RequestError(413, "request_too_large", f"Request body exceeds the {limit}-byte limit.")
    content_type = (handler.headers.get("Content-Type") or "").split(";", 1)[0].strip().lower()
    if content_type != "application/json":
        raise RequestError(415, "unsupported_media_type", "Content-Type must be application/json.")
    raw = handler.rfile.read(length)
    if len(raw) != length:
        raise RequestError(400, "incomplete_body", "Request body ended before Content-Length bytes were read.")
    try:
        data = json.loads(
            raw.decode("utf-8"),
            parse_constant=_reject_nonfinite_json,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise RequestError(400, "malformed_json", "Request body is not valid UTF-8 JSON.") from exc
    if not isinstance(data, dict):
        raise RequestError(400, "invalid_body", "The JSON request body must be an object.")
    return data


def require_field(body: Mapping[str, Any], name: str) -> Any:
    if name not in body:
        raise RequestError(400, "missing_field", f"Required field {name!r} is missing.", {"field": name})
    return body[name]


def validate_request_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise RequestError(
            400,
            "invalid_request_id",
            "Field 'id' must be a non-empty string of at most 128 characters.",
            {"field": "id"},
        )
    return value.strip()


class RequestIdGuard:
    """Bounded process-local at-most-once guard for mutating HTTP requests."""

    def __init__(self, capacity: int = 1024, ttl_s: float = 30.0) -> None:
        self.capacity = max(32, capacity)
        self.ttl_s = max(5.0, ttl_s)
        self._lock = threading.Lock()
        self._order: deque[tuple[tuple[str, str], float]] = deque()
        self._seen: set[tuple[str, str]] = set()

    def claim(self, path: str, request_id: Optional[str]) -> None:
        if request_id is None:
            return
        key = (path, request_id)
        now = time.monotonic()
        with self._lock:
            while self._order and now - self._order[0][1] >= self.ttl_s:
                old_key, _ = self._order.popleft()
                self._seen.discard(old_key)
            if key in self._seen:
                raise RequestError(
                    409,
                    "duplicate_request",
                    "This request id was already accepted; the action will not run again.",
                    {"id": request_id, "path": path},
                )
            self._seen.add(key)
            self._order.append((key, now))
            # Capacity is a soft bound: entries inside the protocol's minimum
            # replay window are never evicted, even during a request burst.
            while len(self._order) > self.capacity and now - self._order[0][1] >= self.ttl_s:
                old_key, _ = self._order.popleft()
                self._seen.discard(old_key)


def finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RequestError(400, "invalid_type", f"Field {name!r} must be a number.", {"field": name})
    result = float(value)
    if not math.isfinite(result):
        raise RequestError(400, "value_out_of_range", f"Field {name!r} must be finite.", {"field": name})
    return result


def nonempty_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RequestError(400, "invalid_type", f"Field {name!r} must be a non-empty string.", {"field": name})
    return value.strip()


def number_list(value: Any, name: str, *, length: Optional[int] = None) -> list[float]:
    if not isinstance(value, list):
        raise RequestError(400, "invalid_type", f"Field {name!r} must be an array.", {"field": name})
    if length is not None and len(value) != length:
        raise RequestError(
            400,
            "value_out_of_range",
            f"Field {name!r} must contain exactly {length} entries.",
            {"field": name, "expected_length": length},
        )
    return [finite_number(item, f"{name}[{index}]") for index, item in enumerate(value)]


# Readable aliases used by controller-side adapters. Keep the canonical names
# above stable for the pip package while letting older in-tree bridges migrate
# without carrying another implementation.
bearer_token = configured_token
trusted_origins_from_env = allowed_origins
require_authorized = check_authorization
validate_bind_host = validate_bind
