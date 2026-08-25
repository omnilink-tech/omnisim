# The Omni Key, and driving OmniLink from code

**Who this is for.** You have an OmniSim clone and you want the chat demos to
think with a real model, or you want to call the OmniLink platform directly
from your own program.

**Where this came from.** The OmniLink website used to carry a twenty-page
documentation site. It was generated in April 2026 and never regenerated, so
it still taught credits, the Bronze/Silver/Gold/Diamond tiers, cloud knowledge
upload and long-term memory — all four of which were removed from the product.
It shipped with a banner at the top admitting it disagreed with the pricing
page. It has been deleted. This page is the part of it that was worth keeping:
the Omni Key, and the HTTP surface. Everything else about *using* OmniLink is
better answered by the repo you are standing in — see
[omnilink-chat-demos.md](omnilink-chat-demos.md),
[omnilink-add-your-robot.md](omnilink-add-your-robot.md) and
[omnilink-build-agent.md](omnilink-build-agent.md).

---

## 1. You may not need a key

Say this first because the old docs did not. **The chat demos work with no
key and no account.** Every bridge falls back down a three-tier ladder
(`setup_omnilink_relay()`, `omnilink_mobile_bridge.py`):

| you have | what drives the robot | what it costs |
|---|---|---|
| `OMNI_KEY` | OmniLink cloud routing (Gemini by default) | free tier; your own provider key for the model |
| a local Ollama, no key | that local model | nothing, nothing leaves the machine |
| neither | offline regex intent router, nine patterns | nothing |

The regex router is genuinely *better* than the agent on the phrases it
covers — it answers in about 0.2 s and never hallucinates a completion. The
key buys you paraphrase, multi-step plans, questions about state, and the
platform features in §3. It is an upgrade, not a prerequisite.

## 2. Getting a key

```bash
python -m omnisim key
```

That prints your current status and, if you have no key, the three steps —
including the exact line for the shell you are actually in, which is the part
people got wrong.

```bash
python -m omnisim key --open     # open the key page in a browser
python -m omnisim key --all      # the set-the-variable line for every shell
python -m omnisim key --check    # ask the platform whether your key works
```

Manually, it is: sign in at **<https://www.omnilink-agents.com/key>** (Google
or GitHub — those are the two ways in), press *Generate API key*, copy the
value. It starts with `olink_`.

Then set it for the session:

| shell | line |
|---|---|
| PowerShell | `$env:OMNI_KEY = "olink_..."` |
| cmd.exe | `set OMNI_KEY=olink_...` |
| bash / zsh | `export OMNI_KEY="olink_..."` |
| fish | `set -x OMNI_KEY "olink_..."` |

`OMNI_KEY` is read **only from the environment** — there is no config file, no
dotenv and no CLI flag (`relay.py`, `omnilink_enabled()`). A key exported in
one terminal does not exist in another.

### Confirm it took

`--check` calls the platform, so it distinguishes "looks like a key" from
"is a key":

```
$ python -m omnisim key --check
Checking your Omni Key against the platform...
  Key olink_defi...ey was REJECTED by the platform (HTTP 403).
  Platform said: Access denied: invalid Omni Key.
```

In a running world, the proof is the bridge's own startup line:

```
[omnilink_mobile_bridge] OmniLink relay ON (agent='OmniSim-tb3_burger')
```

and the chat panel's status reading `OmniLink relay (g1-engine)` rather than
`local intent (regex)`.

**A key that fails is loud on purpose.** The bridge prints
`!! OmniLink relay setup FAILED` with a traceback and falls back to regex,
because a silent downgrade once let two whole demos run with no LLM in the
loop and nobody noticed.

## 3. What the key actually unlocks

Five things, all gated on the same variable:

1. **Cloud model routing** — `OMNILINK_ENGINE` picks the engine, and takes
   the full name: `g1-engine` (Gemini, the default), `g2-engine` (GPT),
   `g3-engine` (Grok), `g4-engine` (Claude). The platform also runs
   `g5-engine` (local Ollama) and `g6-engine` (OpenRouter); `g5` is what the
   bridge pushes for you in hybrid mode — key set *and* a local Ollama
   running, which gets you free local inference plus platform memory,
   profile sync and telemetry.
2. **Voice** — speech in and out. STT needs the platform; there is no offline
   path for it.
3. **Profile sync** — each robot registers itself as an `OmniSim-<robot_id>`
   agent profile, so it appears in the platform's roster. Disable with
   `OMNILINK_PROFILE_SYNC=0`.
4. **Presence** — the bridge heartbeats every 30 s and declares that interval,
   so the platform sizes its staleness window to the runtime instead of to a
   browser tab. This is what makes a robot read as *online* rather than
   flickering. Disable with `OMNILINK_PRESENCE=0`.
5. **Cross-session memory and usage telemetry** — `OMNILINK_MEMORY`,
   `OMNILINK_USAGE`.

**BYOK — and it is a genuine second step, not a footnote.** `OMNI_KEY`
authenticates *you to the platform*; it is not a model key and the two are not
interchangeable. OmniLink routes, remembers and schedules; your provider does
the inference and bills you directly, at 0% markup on every plan. Until a
provider is connected the platform answers `402 BYOK_REQUIRED` and an agent
cannot think.

You do not need a browser for this. The *API & Keys* page works, but the
endpoint behind it takes an ordinary authenticated call, so the terminal —
or a coding agent working in this clone — can do it too:

```bash
python -m omnisim byok                 # what is connected, and what is missing
python -m omnisim byok --providers     # the options, and which engine each unlocks
python -m omnisim byok --add google    # paste a key; input is hidden
```

Start with **google**: it is the only provider with a free tier and no card
(`AIza...` from aistudio.google.com/apikey), and it is what `g1-engine` uses
by default. `openai`, `anthropic`, `xai` and `openrouter` are all paid and all
optional — connect one only when you want `g2`, `g4`, `g3` or `g6`.

The key is sent straight to the platform, which stores it encrypted. **Never
paste a provider key into a file in this repo** — nothing here reads one, and
a key in a working tree is a key in a commit sooner or later.

## 4. Driving the platform from code

The agent surface is one endpoint.

```
POST https://www.omnilink-agents.com/api/chat
Authorization: Bearer olink_...
Content-Type: application/json
```

```json
{
  "agentName": "OmniSim-husky",
  "message": "drive forward 1 meter",
  "availableToolDetails": [
    {
      "name": "set_velocity",
      "description": "Drive the base.",
      "parameters": {
        "type": "object",
        "properties": { "v": { "type": "number" }, "w": { "type": "number" } }
      }
    }
  ]
}
```

The reply is `{text, toolCalls: [{id, name, arguments}]}`. **You execute the
tool calls** — the platform never touches your robot; it only decides. Append
each result to the history and call again for the next turn. That loop, with
memory, cancellation and presence already handled, is
`OmniLinkRelay.dispatch()` in
[`packages/omnisim-bridges`](../../packages/omnisim-bridges/) — use it rather
than rewriting it.

Auth quirk worth knowing: most engines accept the `Authorization` header;
`g3-engine` reads the key from the body only.

### Python

```bash
pip install omnilink
```

`ToolRunner` runs the same loop for you, and the OmniSim-facing wrapper is
`packages/omnisim-bridges`:

```bash
pip install -e packages/omnisim-bridges/
```

That package is `0.1.0` and **not on PyPI** — install it from the clone. Do
not write `pip install omnisim-bridges` in anything user-facing; it does not
resolve.

## 5. Honest limits

- **Nothing here has run on physical hardware.** The real-robot bridges in
  `agents/bridges/` log the commands they would send, against mock drivers.
  This is a control-surface story, not a sim-to-real claim — see
  [omnilink-sim-to-real.md](omnilink-sim-to-real.md).
- **The platform features are a lock-in surface.** Voice, cross-session
  memory, usage telemetry and profile sync are thin pass-throughs gated on
  `OMNI_KEY`. They do not port to the starter kit for free, and that is the
  fair counterweight to everything above.
- **"OmniLink is free" is not a true sentence unqualified.** The free tier is
  offline regex or local Ollama. Platform features need a key; Claude routing
  needs your own Anthropic key.

## See also

- [omnilink-chat-demos.md](omnilink-chat-demos.md) — talk to a robot, from a
  fresh clone
- [omnilink-add-your-robot.md](omnilink-add-your-robot.md) — add your own, in
  about 50 lines
- [omnilink-build-agent.md](omnilink-build-agent.md) — scaffold an agent
  (`python -m omnisim agent new`)
- [PROTOCOL.md](../../PROTOCOL.md) — the OmniSim Wire Protocol
