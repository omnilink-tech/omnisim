# omnilink_launcher

Supervisor controller for the OmniSim demo launcher world ([`projects/samples/demos/worlds/omnilink_launcher.omniworld`](../../worlds/omnilink_launcher.omniworld)).

## What it does

1. Loads [`demos.json`](demos.json) — the hand-curated catalogue of every demo in the repo.
2. Pushes the catalogue to the launcher's Robot Window ([`resources/projects/plugins/robot_windows/omnilink_launcher/`](../../../../../resources/projects/plugins/robot_windows/omnilink_launcher/)) on the `manifest` channel.
3. Listens for `load:<repo-relative-world-path>` messages from the side panel; resolves the path, sanity-checks it points at a real `.wbt` inside the repo, and calls `Supervisor.worldLoad()` to switch worlds.

## Wire protocol

| Direction | Tag | Payload | Meaning |
|---|---|---|---|
| panel → controller | `ready` | — | handshake; request manifest |
| panel → controller | `load:<rel>` | repo-relative `.wbt` path | switch to this world |
| controller → panel | `manifest:<json>` | full catalogue JSON | catalogue (sent on boot + on `ready`) |
| controller → panel | `loading:<abs>` | absolute path | optimistic ack |
| controller → panel | `status:<text>` | advisory text | informational |
| controller → panel | `error:<text>` | error text | shown in red |

## Adding a new demo

1. Edit [`demos.json`](demos.json) — add an entry to an existing category, or add a new category. The schema is:
   ```json
   {
     "categories": [
       {
         "id": "<short-id>",
         "label": "<Human label>",
         "description": "<one-line description>",
         "demos": [
           {
             "id": "<short-id>",
             "name": "<Human title>",
             "world": "projects/samples/demos/worlds/<your-world>.wbt",
             "blurb": "<one-line description>"
           }
         ]
       }
     ]
   }
   ```
2. Add the matching row to [`/DEMOS.md`](../../../../../DEMOS.md).
3. Reload the launcher world in OmniSim — the new demo appears immediately.

## Why it lives here, not under `agents/`

The launcher is an OmniSim **supervisor controller**, not an OmniLink agent — it speaks OmniSim's `wwiSendText` / `wwiReceiveText` to a side-panel Robot Window, not OmniLink HTTP. The closest parallel in the repo is [`omnilink_chat`](../../plugins/robot_windows/omnilink_chat/) (the chat console used by every `omnilink_<robot>.wbt` demo); the launcher follows the same pattern.
