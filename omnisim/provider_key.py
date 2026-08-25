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

"""`python -m omnisim byok` -- connect the model provider that does the thinking.

An Omni Key is step one of two, and the second step was only ever documented
as "add your own provider key on the platform's API & Keys page". That is a
browser instruction in a tool whose whole premise is that a coding agent can
drive it from a clone. The endpoint behind that page (`/api/user-credentials`)
takes an ordinary authenticated POST, so there was no reason the terminal --
or an agent working on the operator's behalf -- could not do it too.

Without a provider key OmniLink answers `402 BYOK_REQUIRED`: OmniLink routes
and remembers, your provider does the inference, and you pay them directly at
0% markup. The free path is Google AI Studio -- an `AIza...` key, no card.

    python -m omnisim byok                    # what is connected, and what is missing
    python -m omnisim byok --add google       # paste a key (hidden input)
    python -m omnisim byok --add openai --key sk-...   # non-interactive
    python -m omnisim byok --providers        # what can be connected, and why

Stdlib only, like `omnisim key`: this has to run in a clone where nothing has
been pip-installed yet.
"""

from __future__ import annotations

import argparse
import getpass
import json
import os
import sys
import urllib.error
import urllib.request

BASE = os.environ.get("OMNILINK_BASE_URL", "https://www.omnilink-agents.com").rstrip("/")
CREDENTIALS_ENDPOINT = f"{BASE}/api/user-credentials"
_TIMEOUT_S = 20

#: Mirrors VALID_PROVIDERS in api/user-credentials.ts. Kept as data rather
#: than fetched, so `--providers` still explains the options when the network
#: is down or the key is missing.
PROVIDERS: dict[str, dict[str, str]] = {
    "google": {
        "engine": "g1-engine (Gemini)",
        "hint": "AIza...",
        "note": "FREE tier, no card. aistudio.google.com/apikey -- start here.",
    },
    "openai": {
        "engine": "g2-engine (GPT)",
        "hint": "sk-...",
        "note": "platform.openai.com/api-keys -- paid.",
    },
    "xai": {
        "engine": "g3-engine (Grok)",
        "hint": "xai-...",
        "note": "console.x.ai -- paid.",
    },
    "anthropic": {
        "engine": "g4-engine (Claude)",
        "hint": "sk-ant-...",
        "note": "console.anthropic.com -- paid.",
    },
    "openrouter": {
        "engine": "g6-engine (many models)",
        "hint": "sk-or-...",
        "note": "openrouter.ai/keys -- one key, many models.",
    },
}


def _trust() -> None:
    """Hand Python the OS trust store when truststore is present.

    Same reason as `omnisim key`: a corporate/AV TLS interceptor otherwise
    fails verification against the default store.
    """
    try:  # pragma: no cover - environment dependent
        import truststore  # type: ignore[import-not-found]

        truststore.inject_into_ssl()
    except Exception:
        pass


def _request(method: str, url: str, key: str, body: dict | None = None):
    """Returns (status, parsed_json_or_None, error_text)."""
    _trust()
    data = json.dumps(body).encode("utf-8") if body is not None else None
    req = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers={
            "Authorization": f"Bearer {key}",
            "Accept": "application/json",
            **({"Content-Type": "application/json"} if data else {}),
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=_TIMEOUT_S) as resp:
            raw = resp.read().decode("utf-8", "replace") or "{}"
        return resp.status, json.loads(raw), ""
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", "replace")
        try:
            parsed = json.loads(raw or "{}")
            detail = parsed.get("error") or parsed.get("message") or raw
            if isinstance(detail, dict):
                detail = detail.get("message") or json.dumps(detail)
        except Exception:
            parsed, detail = None, raw
        return exc.code, parsed, str(detail)[:300]
    except urllib.error.URLError as exc:
        return 0, None, f"could not reach {BASE} ({exc.reason})"
    except Exception as exc:  # pragma: no cover - defensive
        return 0, None, f"{type(exc).__name__}: {exc}"


def _omni_key() -> str:
    return (os.environ.get("OMNI_KEY") or "").strip()


def _need_key() -> int:
    print("OMNI_KEY is not set, so I cannot see or change your provider keys.")
    print()
    print("  Run `python -m omnisim key` first -- it prints the exact line for")
    print("  your shell. That key identifies your OmniLink account; the")
    print("  provider key this command manages is what pays for the tokens.")
    return 2


def _print_providers() -> int:
    print("Model providers OmniLink can route to. You pay them directly;")
    print("OmniLink adds 0% markup on every plan.")
    print()
    for name, meta in PROVIDERS.items():
        print(f"  {name:<11} {meta['engine']}")
        print(f"              key looks like {meta['hint']}  --  {meta['note']}")
    print()
    print("  Start with google: it is the only one with a free tier and no card.")
    print("  Add one with:  python -m omnisim byok --add google")
    return 0


def _list(key: str) -> int:
    status, body, err = _request("GET", CREDENTIALS_ENDPOINT, key)
    if status != 200:
        print(f"Could not read your provider keys: {err or status}")
        return 1

    creds = (body or {}).get("credentials") or []
    if not creds:
        print("No model provider connected yet.")
        print()
        print("  Until one is, OmniLink answers 402 BYOK_REQUIRED and an agent")
        print("  cannot think. The free path takes about a minute:")
        print()
        print("      python -m omnisim byok --add google")
        print()
        print(f"  (Get the key at aistudio.google.com/apikey -- free, no card.)")
        return 0

    print(f"Connected providers ({len(creds)}):")
    print()
    connected = set()
    for c in creds:
        provider = c.get("provider", "?")
        connected.add(provider)
        bits = [c.get("credential_type", "api_key")]
        if c.get("is_default"):
            bits.append("default")
        state = c.get("status", "?")
        if state != "active":
            bits.append(state.upper())
        used = c.get("usage_count")
        if used:
            bits.append(f"{used} calls")
        engine = PROVIDERS.get(provider, {}).get("engine", "")
        print(f"  {provider:<11} {engine:<24} [{', '.join(str(b) for b in bits)}]")
        fail = c.get("last_failure_reason")
        if fail and c.get("consecutive_failures"):
            print(f"              last failure: {str(fail)[:70]}")

    missing = [p for p in PROVIDERS if p not in connected]
    if missing:
        print()
        print(f"  Not connected: {', '.join(missing)}")
        print(f"  Add one with:  python -m omnisim byok --add {missing[0]}")
    return 0


def _add(key: str, provider: str, api_key: str, label: str) -> int:
    provider = provider.strip().lower()
    if provider not in PROVIDERS:
        print(f"Unknown provider {provider!r}. Choose one of: "
              f"{', '.join(PROVIDERS)}")
        return 2

    if not api_key:
        meta = PROVIDERS[provider]
        print(f"Paste your {provider} key ({meta['hint']}).")
        print(f"  {meta['note']}")
        print("  Input is hidden and the key is sent straight to OmniLink,")
        print("  which stores it encrypted. It is never written to this repo.")
        try:
            api_key = getpass.getpass("  key: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nCancelled.")
            return 1
    if not api_key:
        print("No key given, nothing to do.")
        return 1

    status, body, err = _request(
        "POST", CREDENTIALS_ENDPOINT, key,
        {"provider": provider, "credential_type": "api_key",
         "api_key": api_key, "label": label},
    )
    if status in (200, 201):
        print(f"Connected {provider}. {PROVIDERS[provider]['engine']} is now available.")
        print()
        print("  Check it end to end with:  python -m omnisim key --check")
        return 0

    # The platform's own message is better than anything invented here --
    # it knows why a key was rejected (bad prefix, too short, duplicate).
    print(f"OmniLink would not take that key ({status}).")
    if err:
        print(f"  {err}")
    if status in (401, 403):
        print("  That looks like an OMNI_KEY problem, not a provider-key one.")
        print("  Try:  python -m omnisim key --check")
    return 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="omnisim byok",
        description=(
            "Connect the model provider that does the thinking. An Omni Key "
            "identifies your account; a provider key pays for the tokens. "
            "OmniLink takes 0% markup -- you pay Google, OpenAI, Anthropic, "
            "xAI or OpenRouter directly."
        ),
    )
    parser.add_argument("--providers", action="store_true",
                        help="List the providers you can connect, and which engine each unlocks.")
    parser.add_argument("--add", metavar="PROVIDER",
                        help="Connect a provider (google, openai, xai, anthropic, openrouter).")
    parser.add_argument("--key", metavar="API_KEY", default="",
                        help="The provider key. Omit to be prompted with hidden input.")
    parser.add_argument("--label", default="default",
                        help="A label for this credential (default: 'default').")
    args = parser.parse_args(argv)

    if args.providers:
        return _print_providers()

    omni = _omni_key()
    if not omni:
        return _need_key()

    if args.add:
        return _add(omni, args.add, args.key.strip(), args.label)
    return _list(omni)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
