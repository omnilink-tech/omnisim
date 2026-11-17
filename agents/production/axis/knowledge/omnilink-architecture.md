# OmniLink architecture

Source-of-truth notes on how the OmniLink platform is put together, scoped to what Axis needs to know.

## What OmniLink is

OmniLink is a multi-agent platform for building, running, and interacting with AI agents. Operators create named agent profiles (each with its own system prompt, tools, and memory), and talk to them through a web dashboard or a Python client.

- Production: <https://www.omnilink-agents.com>
- Client library: `omnilink` on PyPI (source in `omnilink-lib/`)
- Repo layout: dashboard in `src/`, server API in `api/`, agent definitions in `agents/`

## Core surfaces relevant to Axis

- **Agent profiles** — a profile is identity + system prompt (`mainTask`) + tool manifest + standing orders + engine selection. Stored in a durable profiles store. Axis's profile lives at `agents/axis/profile.json`.
- **Chat** — `/api/chat` is the single entry point. Axis's responses flow through this.
- **Tools** — agents expose tools via `availableToolDetails` in their profile. A local HTTP server (`toolCallbackUrl`) receives POST /tool calls from the browser and returns results. Axis's local callback runs on port 51516 by default.
- **Standing orders** — profiles declare scheduled activities (`telemetry_tick`, `safety_watchdog`, `session_summary`). The server fires them on schedule and sends the result back through the same chat surface.
- **Memory** — three tiers: short-term (session cache), long-term (a durable memories store OR local markdown), conversations + messages (durable chat history). Axis uses the local markdown variant.

## Engines

Four engines are selectable per chat. Axis defaults to `g2-engine`.

| ID | Provider | Underlying model family |
|---|---|---|
| `g1-engine` | Google Gemini | Gemini 2.x |
| `g2-engine` | OpenAI | GPT-4 class (default) |
| `g3-engine` | xAI | Grok |
| `g4-engine` | Anthropic | Claude 4.x |

Low-temperature motion-command work (Axis's primary surface) is well-served by g2 or g4. g3 has been flagged for factual slips on known-API tables; avoid it for robot-spec lookups.

## Auth model

- **Omni Key** — `olink_*` token. Axis's runner reads it from the `OMNI_KEY` environment variable and uses it as a Bearer header on every `/api/*` request.

## How Axis fits

```
Operator / planner
      |
      v
OmniLink  ──►  Axis agent  ──►  OmniLink automation endpoints
                                        |
                                        v
                                  OmniSim bridge
                                        |
                                        v
                            OmniSim controller (omnilink_arm_bridge.py pattern)
                                        |
                                        v
                                  Simulated OmniArm 6
```

Axis is "just another agent" from OmniLink's perspective — same profile, same chat endpoint, same tool-callback dance. The difference is the tool surface: instead of computer-use, Axis's tools proxy HTTP calls to the OmniSim bridge.

## First-party agents

Defined under `agents/`:

- **OmniLink first-party assistant** — a general-purpose operator agent with knowledge folder and tiered tool manifest.
- **Axis** — robot-control specialist (this agent). Proxies to the OmniSim bridge; reuses the first-party assistant's knowledge + local-memory pattern.
- **Haven** — smart-home scaffold, dormant.
