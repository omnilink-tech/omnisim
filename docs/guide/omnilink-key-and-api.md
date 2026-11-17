# OmniKey and model connections

OmniLink is the connected AI agent experience. Every session requires an
OmniKey, including on Free. A model provider connection supplies inference;
provider usage is billed separately.

1. [Create or sign into your OmniLink account](https://www.omnilink-agents.com/agents/start).
2. Connect your selected model provider through onboarding.
3. Follow the [robot setup guide](omnilink-chat-demos.md) to launch a demo with
   your OmniKey and matching engine selection.

Use `python -m omnisim key` to check key setup and `python -m omnisim byok`
to inspect connected model providers. Keep credentials out of worlds, source
files, recordings and public logs. An initialized connection is not proof that
the selected provider works; confirm with a successful request.

No keyless chat mode or automatic local-model fallback is provided. Local
models connect through OmniLink using an OmniKey and the configured connector.
If authentication or inference fails, fix that connection before sending more
instructions. Direct simulator controls and Stop remain available independently.

For applications integrating the platform, use the published OmniLink client
and [current connections page](https://www.omnilink-agents.com/dashboard?view=api).
For robot-facing endpoints, see the [bridge package](../../packages/omnisim-bridges/)
and [sim-to-real control-surface guide](omnilink-sim-to-real.md).

[v9 scope and release status](../RELEASE_NOTES_v9.md).
