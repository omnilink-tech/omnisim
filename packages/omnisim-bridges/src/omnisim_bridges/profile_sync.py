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

"""profile_sync — push a per-robot agent profile to the OmniLink platform.

Why
---
After this runs, the operator can pick `OmniSim-husky` (or whatever
agent_name is) in the omnilink-agents.com web UI's profile dropdown
and chat to the simulated robot from the platform's own chat surface.
The platform-side chat returns structured `toolCalls`; the user's
browser POSTs them to the profile's `toolCallbackUrl` (the bridge's
`/tool` endpoint on `127.0.0.1`). Browser-to-localhost works because
the bridge explicitly trusts the OmniLink web origin; it never emits a
wildcard CORS origin for robot-control routes.

Idempotent
----------
`ensure_profile` is safe to call on every bridge boot. It does
`list_profiles()`, matches by name, then `update_profile(pid, ...)`
or `create_profile(name, ...)`. Each call returns the profile id —
worth stashing if a downstream caller wants to address the profile
by id elsewhere.

Toggle
------
`OMNILINK_PROFILE_SYNC=0` skips this. `OMNILINK_PROFILE_SYNC` defaults
to `1` only when `OMNI_KEY` is set; without a key, profile sync has
nothing to do anyway.
"""

from __future__ import annotations

import os
import re
from typing import Any, Dict, List, Optional


def agent_name_for(robot_id: str) -> str:
    """The platform identity for one robot -- and how a scratch run avoids it.

    The agent name is the key for BOTH the profile (which carries
    `toolCallbackUrl`) and the conversation memory. So a test or parallel run
    that reuses a production robot id does not merely coexist with the real
    demo, it TAKES IT OVER: measured twice in one day, a verification run on
    scratch ports 8785-8787 (and again on 8805-8807) repointed the live
    `OmniSim-*` profiles at ports that died with it, so any
    platform-initiated tool call afterwards hit a closed socket.

    The memory damage was subtler and worse. Adversarial probes ("you never
    stopped, you're making that up", "I have the CCTV") landed in the REAL
    agents' durable history, including the model's own capitulation and two
    fabrications. That history is replayed as context on every later turn, so
    a demo the operator opens fresh would be primed by a transcript in which
    this robot lies and then folds under pressure.

    Set OMNILINK_AGENT_TAG for any run that is not the shipped demo. It
    suffixes the identity, giving that run its own profile and its own memory
    while leaving production untouched.
    """
    tag = (os.environ.get("OMNILINK_AGENT_TAG") or "").strip()
    base = f"OmniSim-{robot_id}"
    if not tag:
        return base
    safe = re.sub(r"[^A-Za-z0-9_.-]", "-", tag)[:32].strip("-")
    return f"{base}-{safe}" if safe else base


def is_enabled() -> bool:
    if not os.environ.get("OMNI_KEY", "").strip():
        return False
    return os.environ.get("OMNILINK_PROFILE_SYNC", "1").strip() not in (
        "0", "false", "no", "",
    )


def safety_gate_summary(
    surface: Optional[str] = None,
    *,
    gated_paths: Optional[List[str]] = None,
    ungated_paths: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """The `safetyGate` block for the profile -- the same object `/capabilities`
    serves, built by the same function.

    ONE BUILDER, because the whole point of publishing it is that a client
    can compare what the profile promises with what the bridge answers. Two
    builders would be the drift the gate parity work exists to end. Falls
    back to an explicitly UNKNOWN block when `bridge_base` will not import,
    rather than claiming a gate is present.
    """
    try:
        from .bridge_base import safety_gate_block
    except Exception as exc:                  # pragma: no cover - defensive
        return {"present": None, "surface": surface,
                "gated_paths": list(gated_paths or []),
                "ungated_paths": list(ungated_paths or []),
                "note": f"gate block unavailable in this build ({exc})"}
    return safety_gate_block(surface,
                             gated_paths=gated_paths,
                             ungated_paths=ungated_paths)


def build_settings(
    *,
    main_task: str,
    tool_defs: List[Dict[str, Any]],
    engine: str,
    tool_callback_url: Optional[str] = None,
    prompt_callback_url: Optional[str] = None,
    surface: Optional[str] = None,
    actions: Optional[List[str]] = None,
    safety_gate: Optional[Dict[str, Any]] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Construct the `settings` dict pushed into the profile.

    Matches the schema the web UI and the chat handler read:
      mainTask              -- system instruction text
      availableTools        -- comma-separated tool names (legacy)
      availableToolDetails  -- list of {name, description, parameters}
      allowToolUse          -- bool
      engine                -- "g1-engine" / "g2-engine" / ...
      toolCallbackUrl       -- where the UI POSTs tool calls

    And, added for the v9 loop (D3):
      promptCallbackUrl     -- the bridge's `/prompt`, so the platform can
                               hand an operator's SENTENCE to the
                               deterministic parser instead of paying a
                               model to interpret it. The web and unattended
                               loops fall back to `/api/chat` + `/tool` when
                               this is absent or unreachable, so an old
                               platform against a new profile is unaffected.
      surface               -- this bridge's robot class, which is what picks
                               the magnitude rail for a tool two classes
                               share (`move_body{vertical}`: a 120 m climb on
                               a drone, a 1.0 m body shift on a quadruped).
      actions               -- the doors this bridge serves, e.g.
                               ["tool", "prompt"]. Derived from the URLs
                               above when not given: a caller that passed a
                               prompt URL has just told us `/prompt` is
                               served. A NEW FRAME KIND IS DECLARED BEFORE
                               IT IS SENT -- that is what stops a new
                               platform breaking an old bridge.
      safetyGate            -- `safety_gate_summary()`; the same block
                               `/capabilities` serves, including the
                               `ungated_paths` a client needs in order to
                               know which commands it must bound itself.

    Every key here is OPTIONAL and additive. A platform that has never heard
    of them reads the profile exactly as it did before.
    """
    settings: Dict[str, Any] = {
        "mainTask": main_task,
        "availableTools": ", ".join(t.get("name", "") for t in tool_defs),
        "availableToolDetails": list(tool_defs),
        "allowToolUse": True,
        "engine": engine,
    }
    if tool_callback_url:
        settings["toolCallbackUrl"] = tool_callback_url
    if prompt_callback_url:
        settings["promptCallbackUrl"] = prompt_callback_url
    if surface:
        settings["surface"] = surface
    if actions is None:
        derived = []
        if tool_callback_url:
            derived.append("tool")
        if prompt_callback_url:
            derived.append("prompt")
        actions = derived or None
    if actions:
        settings["actions"] = list(actions)
    if safety_gate:
        settings["safetyGate"] = dict(safety_gate)
    if extra:
        settings.update(extra)
    return settings


def ensure_profile(
    client: Any,
    agent_name: str,
    *,
    main_task: str,
    tool_defs: List[Dict[str, Any]],
    engine: str,
    tool_callback_url: Optional[str] = None,
    prompt_callback_url: Optional[str] = None,
    surface: Optional[str] = None,
    actions: Optional[List[str]] = None,
    safety_gate: Optional[Dict[str, Any]] = None,
    extra_settings: Optional[Dict[str, Any]] = None,
) -> Optional[str]:
    """Idempotent profile push. Returns the profile id, or None on error.

    Errors are caught and logged. Profile sync is a convenience — the
    side-menu chat keeps working even when this fails.

    ⚠️ READ `agent_name_for` ABOVE BEFORE ADDING A CALLER. The agent name
    keys the profile AND the memory, so a scratch run that reuses a
    production robot id does not coexist with the live demo, it TAKES IT
    OVER -- measured twice in one day. `promptCallbackUrl` makes that worse
    in exactly one way and no more: there is now a SECOND dead URL to
    repoint, so a scratch run without `OMNILINK_AGENT_TAG` now silently
    breaks the live demo's sentence door as well as its tool door. The fix
    is unchanged and is still the only fix: tag the run.
    """
    try:
        settings = build_settings(
            main_task=main_task,
            tool_defs=tool_defs,
            engine=engine,
            tool_callback_url=tool_callback_url,
            prompt_callback_url=prompt_callback_url,
            surface=surface,
            actions=actions,
            safety_gate=safety_gate,
            extra=extra_settings,
        )
        profiles = client.list_profiles()
        existing = next(
            (p for p in profiles if (p.get("name") or "").lower() == agent_name.lower()),
            None,
        )
        if existing:
            pid = existing.get("id", "")
            client.update_profile(pid, name=agent_name, settings=settings)
            print(f"[profile_sync] updated profile {agent_name!r} (id={pid})")
            return pid
        result = client.create_profile(agent_name, settings=settings)
        pid = result.get("id", "")
        print(f"[profile_sync] created profile {agent_name!r} (id={pid})")
        return pid
    except Exception as e:
        print(f"[profile_sync] skipped ({e})")
        return None
