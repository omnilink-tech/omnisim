# OmniSim Picker — manipulation specialist

An OmniLink profile-only agent specialised for pick-and-place on a
single OmniSim arm. It reuses the existing arm bridge's `/tool`
endpoint, so there's no separate Python process to keep running —
just register the profile once and chat from the web UI.

## What's different from the per-arm default agent

The default `OmniSim-<robot_id>` profile (auto-pushed by every arm
bridge on boot) has a generic system prompt. The Picker swaps that
prompt for one tuned to manipulation tasks:

- Always preview targets with `solve_ik` before motion.
- Approach pre-grasp from above (10 cm clearance), descend, close
  gripper, lift back to clearance.
- Mirror that pattern for placement.
- Refuse unreachable targets with an explanation.
- Always confirm grasp completion via `get_robot_state` before lifting.

The tool surface is identical to the per-arm default — Picker is
purely a prompt + behavior override.

## How to register

```bash
# 1. Start an arm world in OmniSim (the OmniArm 6 chat world has IK
#    pre-baked):
#    File → Open World → projects/samples/demos/worlds/chat/omnilink_omniarm6.omniworld

# 2. Register the profile (idempotent -- safe to re-run):
export OMNI_KEY="olink_..."
# default PICKER_BRIDGE is http://127.0.0.1:8765 (matches the single-arm worlds)
pip install omnilink
python agents/templates/picker/register.py

# 3. Open https://omnilink-agents.com, pick "OmniSim-Picker", chat.
```

To remove the profile:

```bash
python agents/templates/picker/register.py --delete
```

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `OMNI_KEY` | _required_ | OmniLink API key. |
| `PICKER_BRIDGE` | `http://127.0.0.1:8765` | URL of the arm bridge whose `/tool` endpoint receives the platform's tool calls. |

## Pointing at a real robot

Replace `PICKER_BRIDGE` with the URL of any arm bridge that speaks the
same `/tool` schema (`{"tool": "<name>", ...args}` → `{"status", "tool",
"result"}`). The Picker doesn't know or care that it's been driving a
simulated arm — same system prompt, same tool surface, real robot.
