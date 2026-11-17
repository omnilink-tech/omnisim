# Local models through OmniLink

Local inference requires an OmniKey and the configured OmniLink local-model
connector. The robot demos do not connect directly to an unauthenticated local
model or fall back to basic command interpretation.

Two things have to be true before a local model answers anything here:

1. `OMNI_KEY` is set to your OmniKey. It is required on every plan, including
   Free, and the access check runs before the sentence is interpreted. Without
   it the bridge answers `401 omnikey_required` and nothing actuates.
2. `OMNILINK_ENGINE` explicitly names the local engine you connected during
   onboarding. Nothing is selected automatically, and a local server that
   happens to be listening is never picked up on its own.

If either is missing, or the connection fails mid-session, that is an error to
fix — not a switch to a different answerer.

Follow the [current setup guide](../../../../../docs/guide/omnilink-chat-demos.md)
and [OmniLink onboarding](https://www.omnilink-agents.com/agents/start). Direct
simulator controls and Stop remain available independently of the AI connection.
