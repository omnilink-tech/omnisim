"""Privacy scrub for conformance output.

The install-conformance fingerprint and report are designed to be readable —
and ultimately pasted into a public GitHub issue. The raw fingerprint carries
absolute paths (``engine_log``, ``OMNISIM_HOME``) that on most machines embed
the OS username (``C:\\Users\\<name>\\...``), so anything that could leave the
machine goes through here first.

This is the Phase-1 down-payment on the scrub-by-default report bundle
described in docs/developer/install-conformance.md §6.1. It is deliberately
conservative: redact home directories / usernames, leave everything else.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

# C:\Users\<name>\...  or  C:/Users/<name>/...   ->  keep the prefix, drop the name.
_WIN_USER = re.compile(r"([A-Za-z]:[\\/]+Users[\\/]+)[^\\/]+", re.IGNORECASE)
# /Users/<name>/...  (macOS)  and  /home/<name>/...  (linux)
_NIX_USER = re.compile(r"((?:/Users|/home)/)[^/]+")


def _home() -> str | None:
    try:
        return str(Path.home())
    except Exception:
        return None


def _variants(path: str) -> set[str]:
    """A path and its separator-flipped forms (Windows logs mix \\ and /)."""
    return {path, path.replace("\\", "/"), path.replace("/", "\\")}


def scrub_text(text: str) -> str:
    """Redact usernames / local home dirs from a single string."""
    if not text:
        return text
    out = text
    home = _home()
    if home:
        for variant in _variants(home):
            out = out.replace(variant, "~")
    omni = os.environ.get("OMNISIM_HOME")
    if omni:
        for variant in _variants(omni):
            out = out.replace(variant, "<OMNISIM_HOME>")
    out = _WIN_USER.sub(r"\1<user>", out)
    out = _NIX_USER.sub(r"\1<user>", out)
    return out


def scrub(obj):
    """Recursively scrub strings inside a JSON-like structure (returns a copy)."""
    if isinstance(obj, str):
        return scrub_text(obj)
    if isinstance(obj, dict):
        return {k: scrub(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [scrub(v) for v in obj]
    return obj
