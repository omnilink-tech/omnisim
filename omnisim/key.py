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

"""`python -m omnisim key` -- get, set and check your OmniLink Omni Key.

The Omni Key is the one thing OmniSim cannot do for you. Everything else in
a fresh clone works offline: with no key the chat bridges fall back to a
local Ollama model, and failing that to a regex intent router, so the demos
run with no account at all. The key is what upgrades them to a real agent --
cloud model routing, voice, cross-session memory, usage telemetry, and the
profile sync that makes each robot show up on the platform.

This command exists because "sign up and generate a key" used to be one
sentence in a guide, and the operator was then left to work out which of
three shells they were in and what the variable was called. It prints the
line for the shell you are actually standing in, and `--check` asks the
platform whether the key works rather than only whether it looks right.

Stdlib only, on purpose -- it has to run in a clone where nothing has been
pip-installed yet.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
import webbrowser

#: Short path that redirects to the key page, so this string stays typeable.
KEY_PAGE = "https://www.omnilink-agents.com/key"
#: Cheapest authenticated endpoint on the platform: 200 with usage when the
#: key is good, 401 when it is not. Used only to answer "does this work?".
USAGE_ENDPOINT = "https://www.omnilink-agents.com/api/omni-key-usage"
KEY_PREFIX = "olink_"
PLACEHOLDER = f"{KEY_PREFIX}YOUR_KEY_HERE"
_TIMEOUT_S = 10


def _detect_shell() -> str:
    """Best-effort guess at the shell the operator is typing into.

    Wrong guesses are cheap here -- every form is printed under `--all`, and
    the guess only decides which one gets top billing.
    """
    if os.name == "nt":
        # PowerShell defines PSModulePath and cmd.exe does not. Not airtight
        # (a PowerShell-launched cmd inherits it), but right far more often
        # than assuming one or the other.
        return "powershell" if os.environ.get("PSModulePath") else "cmd"
    shell = os.path.basename(os.environ.get("SHELL", "") or "")
    return shell if shell in ("bash", "zsh", "fish", "sh") else "bash"


def _set_line(shell: str, key: str = PLACEHOLDER) -> str:
    """The one line that sets OMNI_KEY for `shell`, for this session."""
    if shell == "powershell":
        return f'$env:OMNI_KEY = "{key}"'
    if shell == "cmd":
        return f"set OMNI_KEY={key}"
    if shell == "fish":
        return f'set -x OMNI_KEY "{key}"'
    return f'export OMNI_KEY="{key}"'


_SHELL_LABELS = {
    "powershell": "PowerShell",
    "cmd": "cmd.exe",
    "bash": "bash / zsh",
    "zsh": "bash / zsh",
    "sh": "bash / zsh",
    "fish": "fish",
}


def _redact(key: str) -> str:
    """Show enough of a key to recognise it, never enough to use it."""
    if len(key) <= len(KEY_PREFIX) + 4:
        return f"{KEY_PREFIX}..."
    return f"{key[:len(KEY_PREFIX) + 4]}...{key[-2:]}"


def _print_get_instructions(shell: str) -> None:
    print(f"  1. Get a key:  {KEY_PAGE}")
    print("     Sign in with Google or GitHub, then press Generate API key.")
    print(f"     It starts with `{KEY_PREFIX}`.")
    print("     (`python -m omnisim key --open` opens it for you.)")
    print()
    print(f"  2. Set it for this shell -- {_SHELL_LABELS.get(shell, shell)}:")
    print()
    print(f"       {_set_line(shell)}")
    print()
    print("     Other shells: `python -m omnisim key --all`")
    print()
    print("  3. Connect a model provider -- REQUIRED, and a separate step:")
    print()
    print("       python -m omnisim byok --add google   # free tier, no card")
    print()
    print("     The key above identifies your account; a provider key pays")
    print("     for the tokens (OmniLink takes 0% and never resells them).")
    print("     Without one, chats come back 402 BYOK_REQUIRED.")
    print("     `python -m omnisim byok --providers` lists the options.")
    print()
    print("  4. Confirm both halves:")
    print()
    print("       python -m omnisim key --check")


def _check(key: str) -> int:
    """Ask the platform whether this key is real. Returns an exit code."""
    # Corporate/AV TLS interception used to break verification here. If
    # truststore is installed we hand Python the OS trust store, which sees
    # such a certificate; without it we carry on with the default store.
    try:  # pragma: no cover - environment dependent
        import truststore  # type: ignore[import-not-found]

        truststore.inject_into_ssl()
    except Exception:
        pass

    req = urllib.request.Request(
        USAGE_ENDPOINT,
        headers={"Authorization": f"Bearer {key}", "Accept": "application/json"},
        method="GET",
    )
    try:
        with urllib.request.urlopen(req, timeout=_TIMEOUT_S) as resp:
            raw = resp.read().decode("utf-8", "replace") or "{}"
    except urllib.error.HTTPError as exc:
        # Measured against the live platform 2026-07-23:
        #   403 {"error":"Access denied: invalid Omni Key."}  -- key unknown
        #   401 {"error":"Missing Omni Key."}                 -- header absent
        # We only get here with a key set, so both mean the platform would
        # not take it. Anything else is the platform having a bad day and is
        # NOT evidence about the key.
        if exc.code in (401, 403):
            detail = ""
            try:
                detail = (json.loads(exc.read().decode("utf-8", "replace")) or {}).get("error", "")
            except Exception:
                pass
            print(f"  Key {_redact(key)} was REJECTED by the platform (HTTP {exc.code}).")
            if detail:
                print(f"  Platform said: {detail}")
            print("  It may have been revoked, or copied incompletely.")
            print(f"  Generate a fresh one at {KEY_PAGE}")
            return 1
        print(f"  Could not verify the key -- the platform answered HTTP {exc.code}.")
        print("  That is a platform-side answer, not a verdict on your key.")
        return 2
    except urllib.error.URLError as exc:
        print(f"  Could not reach {USAGE_ENDPOINT}: {exc.reason}")
        print("  This says nothing about whether the key is valid.")
        return 2

    try:
        body = json.loads(raw)
    except ValueError:
        body = {}

    print(f"  Key {_redact(key)} is VALID.")
    label = (body.get("omniKey") or {}).get("label")
    if label:
        print(f"  Label: {label}")
    plan = body.get("plan") or {}
    if plan.get("id"):
        if plan.get("resolved"):
            # e.g. "Plan: Agent Builder ($10 per month)" or "Plan: Free"
            name = plan.get("name") or plan["id"]
            price = plan.get("price")
            frequency = plan.get("frequency")
            suffix = f" ({price} {frequency})" if price and frequency and price != "$0" else ""
            print(f"  Plan: {name}{suffix}")
            rpm = plan.get("requestsPerMinute")
            rpd = plan.get("requestsPerDay")
            if rpm is not None:
                # requestsPerDay == 0 means the tier has no daily cap.
                daily = "no daily cap" if not rpd else f"{rpd} / day"
                print(f"    {rpm} requests / min -- {daily}")
            agents = plan.get("persistentAgentLimit")
            if agents is not None:
                print(f"    Persistent agents: {agents}")
        else:
            print("  Plan: could not be resolved right now (not a verdict on your key).")
    # STEP TWO. The Omni Key identifies the account; it does not pay for tokens.
    # The platform resolves model credentials with no system fallback, so a key
    # with no provider connected gets a 402 BYOK_REQUIRED on the first message.
    # `quota` lists the connected providers -- but it is OMITTED (not empty) on
    # one platform branch, so only an explicitly empty list proves "none".
    quota = body.get("quota")
    print()
    if isinstance(quota, list) and not quota:
        print("  NEXT STEP -- no model provider is connected yet.")
        print("  The Omni Key identifies your account; a provider key pays for")
        print("  the tokens (OmniLink takes 0% and never resells them). Without")
        print("  one, your first message comes back 402 BYOK_REQUIRED.")
        print()
        print("    python -m omnisim byok --add google   # free tier, no card")
        print("    python -m omnisim byok --providers    # the other options")
    else:
        if isinstance(quota, list):
            names = ", ".join(sorted({str(q.get("provider") or "?") for q in quota}))
            print(f"  Model providers connected: {names}")
        else:
            print("  Model providers: could not be read (not a verdict on your key).")
            print("  If a chat returns 402 BYOK_REQUIRED, run: python -m omnisim byok")
        print()
        print("  The chat bridges will route through OmniLink on the next launch:")
        print("    python -m omnisim run-world \\")
        print("      projects/samples/demos/worlds/chat/omnilink_husky.omniworld")
    print()
    print("  Look for this line in the log -- it is the proof the relay is on:")
    print("    [omnilink_mobile_bridge] OmniLink relay ON (agent='OmniSim-husky')")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="omnisim key",
        description=(
            "Get, set and check the OmniLink Omni Key. Without a key the "
            "chat demos still run -- on a local Ollama model, or an offline "
            "regex router -- so this is an upgrade, not a prerequisite."
        ),
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the key page in your browser.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Ask the platform whether the current OMNI_KEY works.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Print the set-the-variable line for every shell.",
    )
    args = parser.parse_args(argv if argv is not None else [])

    key = os.environ.get("OMNI_KEY", "").strip()
    shell = _detect_shell()

    if args.all:
        print("Set OMNI_KEY for the current session:")
        print()
        for name in ("powershell", "cmd", "bash", "fish"):
            print(f"  {_SHELL_LABELS.get(name, name):<12} {_set_line(name)}")
        print()
        print("To make it permanent, put the line in your shell profile, or"
              " use setx on Windows.")
        return 0

    if args.open:
        print(f"Opening {KEY_PAGE}")
        if not webbrowser.open(KEY_PAGE):
            print("Could not open a browser -- copy the URL above instead.")
        print()

    if args.check:
        if not key:
            print("OMNI_KEY is not set, so there is nothing to check.")
            print()
            _print_get_instructions(shell)
            return 1
        print("Checking your Omni Key against the platform...")
        return _check(key)

    # Default: report where the operator actually stands.
    if key:
        print(f"OMNI_KEY is set  ({_redact(key)}).")
        if not key.startswith(KEY_PREFIX):
            print()
            print(f"  ! It does not start with `{KEY_PREFIX}`. Omni Keys do --"
                  " this is probably")
            print("    a provider key (Gemini, OpenAI, ...) in the wrong"
                  " variable. Those go in")
            print("    the platform's API & Keys page, not OMNI_KEY.")
        print()
        print("  Verify it actually works:  python -m omnisim key --check")
        return 0

    print("OMNI_KEY is not set.")
    print()
    print("  The chat demos still work without it -- they fall back to a local")
    print("  Ollama model if one is running, and to an offline regex router if")
    print("  not. A key upgrades them to a real agent: cloud models, voice,")
    print("  memory that survives a restart, and your robots showing up on the")
    print("  platform. Free tier, no card.")
    print()
    _print_get_instructions(shell)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main(sys.argv[1:]))
