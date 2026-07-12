# OmniSim Roomba — mobile-base patroller

An OmniLink profile-only agent specialised for waypoint loops and
return-to-dock on a single OmniSim wheeled base. Same pattern as the
Picker: reuses the mobile bridge's `/tool` endpoint, so no separate
Python process is needed.

## Behaviours baked into the prompt

- "patrol" / "sweep" / "clean" → rectangular 4-waypoint loop.
- "dock" / "park" / "return home" → `reset_to_home` (teleport to spawn).
- "forward N m", "turn left 90" → direct `drive_forward` / `turn`.
- "stop" / "halt" → `stop_robot` then ask what next.
- Refuses targets outside the obvious arena (~±3 m).

## How to register

```bash
# 1. Start a mobile world in OmniSim. Any of these works:
#    File → Open World → projects/samples/demos/worlds/chat/omnilink_husky.wbt
#    (or omnilink_jackal.wbt / omnilink_tb3_burger.wbt / etc.)

# 2. Register the profile:
export OMNI_KEY="olink_..."
# default ROOMBA_BRIDGE is http://127.0.0.1:8765
pip install omnilink
python agents/templates/roomba/register.py

# 3. Open https://omnilink-agents.com, pick "OmniSim-Roomba", chat.
```

To remove the profile:

```bash
python agents/templates/roomba/register.py --delete
```

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `OMNI_KEY` | _required_ | OmniLink API key. |
| `ROOMBA_BRIDGE` | `http://127.0.0.1:8765` | URL of the mobile bridge whose `/tool` endpoint receives the platform's tool calls. |
