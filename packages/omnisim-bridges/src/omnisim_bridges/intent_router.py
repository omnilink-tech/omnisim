# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

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
from typing import Any, Callable, List, Tuple


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
