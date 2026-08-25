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

"""IntentRouter -- tiny regex-based offline fallback router.

Pre-LLM fallback so a bridge's /prompt endpoint does something useful
even without OmniLink configured. Subclass and register your intents
via `register(pattern, handler)` -- the handler returns a dict the
bridge can shape into its response.

This is intentionally minimal. For anything beyond demo intents, plug
in `OmniLinkRelay` instead.

Example:

    from omnisim_bridges import IntentRouter

    class ArmRouter(IntentRouter):
        def __init__(self, bridge):
            super().__init__()
            self.bridge = bridge
            self.register(r"\\bstop\\b",
                          lambda m: ("stop", self.bridge.act_stop()))
            self.register(r"\\bhome\\b",
                          lambda m: ("reset", self.bridge.act_reset_to_home()))

    router = ArmRouter(my_bridge)
    intent, result = router.dispatch("go home")
"""

from __future__ import annotations

import re
from typing import Any, Callable, Dict, List, Optional, Tuple

# ── Shared conversational intents ────────────────────────────────────
#
# Two intents every autonomous bridge needs and that no bridge should
# spell its own way:
#
#   RESUME  -- the counterpart to "stop". A bridge that can be halted by
#              chat but not restarted by chat is a dead end for the
#              operator: the idle loop stays parked until an invisible
#              quiet-window timer expires. Offline routers must check
#              this FIRST, because several resume phrasings collide with
#              motion verbs ("back to work" contains "back", which a
#              mobile router otherwise reads as "reverse 1 m").
#
#   STATUS  -- "what are you doing right now?". Answered from the
#              bridge's own /state dict, never from a template, so the
#              offline reply is as grounded as the LLM one.

RESUME_RE = re.compile(
    r"\b(resume|carry\s+on|carrying\s+on|keep\s+(going|working|at\s+it)|"
    r"continue|as\s+you\s+were|proceed|back\s+to\s+(work|it)|"
    r"get\s+(back\s+to|on)\s+(work|it)|go\s+back\s+to\s+(work|what\s+you\s+were)|"
    r"restart\s+(your\s+)?(work|loop|autonomy)|un[- ]?pause)\b",
    re.IGNORECASE,
)

STATUS_RE = re.compile(
    r"(\b(status|sitrep|telemetry|report)\b|"
    r"\bwhat\s+(are|were)\s+you\s+(doing|up\s+to|working\s+on)|"
    r"\bwhat'?re\s+you\s+doing|\bwhat\s+are\s+you\s+busy\s+with|"
    r"\bdoing\s+right\s+now|\bwhat'?s\s+going\s+on|"
    r"\bhow'?s\s+(it|work|the\s+line)\s+going|\bare\s+you\s+busy)",
    re.IGNORECASE,
)


def is_resume(text: str) -> bool:
    """True when the operator is handing autonomy back to the robot."""
    return bool(RESUME_RE.search(text or ""))


def is_status(text: str) -> bool:
    """True when the operator is asking what the robot is doing."""
    return bool(STATUS_RE.search(text or ""))


def describe_state(state: Optional[Dict[str, Any]]) -> str:
    """One honest English sentence describing a bridge /state dict.

    Reads ONLY what the dict actually contains -- every clause below is
    gated on its key being present -- so an offline reply can never
    claim a job the robot is not doing. Understands the two shapes the
    OmniSim bridges publish (arm: gripper/line; mobile: carrying/tow
    legs) plus the `idle_loop` block they share.
    """
    if not isinstance(state, dict):
        return "I have no state to report."

    who = str(state.get("id") or "this robot")
    bits: List[str] = []

    loop = state.get("idle_loop") or {}
    paused = bool(loop.get("paused"))
    line = state.get("line") or {}

    # ── What am I doing this instant ─────────────────────────────
    if line.get("active"):
        # Arm running as the line master: the box is the real answer.
        box = line.get("fill_box") or "no box"
        fill_state = str(line.get("fill_state") or "?")
        placed = line.get("placed")
        target = line.get("target")
        if placed is not None and target is not None:
            bits.append(f"filling {box} ({placed} of {target} parts in, {fill_state})")
        else:
            bits.append(f"working {box} ({fill_state})")
        if line.get("loaded"):
            bits.append(f"cart {line['loaded']} is loaded")
        if line.get("queued"):
            bits.append(f"{line['queued']} queued on the outfeed")
    elif "carrying" in state:
        # Mobile tug.
        leg = str(loop.get("leg") or state.get("mode") or "idle")
        cart = state.get("carrying")
        if cart:
            bits.append(f"towing {cart} ({leg} leg)")
        else:
            bits.append(f"not towing anything right now ({leg} leg)")
    elif state.get("mode"):
        bits.append(f"in {state['mode']} mode")

    grip = state.get("gripper") or {}
    if grip.get("holding"):
        bits.append("holding a part in the gripper")

    # ── Is my autonomy running ───────────────────────────────────
    if loop:
        loop_mode = str(loop.get("mode") or "idle")
        counter = loop.get("picks")
        unit = "picks"
        if counter is None:
            counter = loop.get("cycles")
            unit = "cycles"
        done = f", {counter} {unit} done" if counter is not None else ""
        if paused:
            bits.append(f"my {loop_mode} loop is PAUSED by an operator command{done}")
        else:
            bits.append(f"my {loop_mode} loop is running{done}")
    else:
        bits.append("I have no autonomous loop running")

    return f"I'm {who}: " + "; ".join(bits) + "."


class IntentRouter:
    """Map free-text prompts to (intent_name, action_result) tuples."""

    def __init__(self) -> None:
        self._rules: List[Tuple[re.Pattern, Callable[[re.Match], Tuple[str, Any]]]] = []

    def register(
        self,
        pattern: str,
        handler: Callable[[re.Match], Tuple[str, Any]],
        flags: int = re.IGNORECASE,
    ) -> None:
        """Add a (regex, handler) pair. First match wins on dispatch."""
        self._rules.append((re.compile(pattern, flags), handler))

    def dispatch(self, text: str) -> Tuple[str, Any]:
        """Try each registered pattern in order. Returns (intent_name,
        action_result). If nothing matches, returns ("unknown", text)."""
        s = (text or "").strip()
        if not s:
            return ("empty", None)
        for pat, handler in self._rules:
            m = pat.search(s)
            if m is not None:
                return handler(m)
        return ("unknown", text)
