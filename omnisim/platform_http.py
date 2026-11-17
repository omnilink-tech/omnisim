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

"""What every OmniSim CLI request to the OmniLink platform must carry.

Measured 2026-11-17: Cloudflare's edge in front of www.omnilink-agents.com
(and staging) refuses Python's default `Python-urllib/x.y` User-Agent with
HTTP 403 "error code: 1010" (Browser Integrity Check) before the platform
sees the request. So `key --check` called every key, valid or not, REJECTED.

Two rules follow, and both live here so no caller has to remember them:

* send an explicit User-Agent (`user_agent()`), and
* a 401/403 is a verdict on the key only when the PLATFORM said it
  (`is_platform_verdict()`): the edge's 1010 comes back as plain text, or,
  when the request asks for JSON, as an RFC 9457 problem body marked
  `"cloudflare_error": true` with no `error` key.

Stdlib only, like the commands that use it.
"""

from __future__ import annotations

import json
import platform
from typing import Any, Dict, Optional, Union


def user_agent(component: str) -> str:
    """e.g. `omnisim/10.1.0 (key; Python 3.12.4; Windows)`."""
    try:
        from omnisim import __version__ as version
    except Exception:  # pragma: no cover - partial checkout
        version = "0"
    return (f"omnisim/{version} ({component}; Python {platform.python_version()}; "
            f"{platform.system() or 'unknown'})")


def headers(component: str, key: str = "", *, json_body: bool = False,
            extra: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    """The headers a platform request sends: User-Agent, Accept, and the key."""
    h = {"User-Agent": user_agent(component), "Accept": "application/json"}
    if key:
        h["Authorization"] = f"Bearer {key}"
    if json_body:
        h["Content-Type"] = "application/json"
    h.update(extra or {})
    return h


def _as_dict(raw: Union[bytes, str, None]) -> Dict[str, Any]:
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", "replace")
    try:
        d = json.loads(raw or "")
    except ValueError:
        return {}
    return d if isinstance(d, dict) else {}


def is_platform_verdict(status: int, raw: Union[bytes, str, None]) -> bool:
    """True when a 401/403 came from the platform itself.

    The platform answers `{"error": "..."}` or `{"error": {"code", "message"}}`.
    Anything else -- Cloudflare's 1010 page, a proxy, a firewall -- says
    nothing about the key.
    """
    if status not in (401, 403):
        return False
    d = _as_dict(raw)
    return bool(d.get("error")) and not d.get("cloudflare_error")


def describe_block(status: int, raw: Union[bytes, str, None]) -> str:
    """One line naming what refused a request that was NOT a platform verdict."""
    d = _as_dict(raw)
    if d.get("cloudflare_error") or d.get("error_code"):
        what = d.get("error_name") or d.get("title") or ""
        return (f"HTTP {status} from Cloudflare, not the platform "
                f"(error {d.get('error_code', '?')}{': ' + what if what else ''})")
    text = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else (raw or "")
    text = " ".join(text.split())[:80]
    return f"HTTP {status} from something in front of the platform: {text!r}"
