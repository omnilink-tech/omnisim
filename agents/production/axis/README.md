# Axis

[![Try in cloud](https://img.shields.io/badge/Try%20in%20cloud-omnilink--agents.com-F6E905?labelColor=000000)](https://www.omnilink-agents.com)
[![One-command run](https://img.shields.io/badge/run-python%20scripts%2Fdev%2Fomnisim__run__agent.py%20--agent%20axis-5DADE2?labelColor=000000)](../../../scripts/dev/omnisim_run_agent.py)

**Mission.** Axis is OmniLink's robot-control agent. It commands robots running inside **OmniSim** — the OmniLink simulation environment built on Webots — through OmniLink's automation HTTP endpoints. It takes operator intents like "move the OmniArm 6 tool to (0.4, 0.2, 0.3)" or "hold at home" and translates them into safe, bounded joint-space or task-space commands, monitors telemetry, and reports back.

**Status.** First-party agent. Prompt, profile, tools, and architecture are source-controlled here, alongside the generic [`omnilink_arm_bridge`](../../../projects/samples/demos/controllers/omnilink_arm_bridge/) controller it drives (launched with `controllerArgs ["--robot" "omniarm6" ...]`) and the OmniArm 6 kinematics it assumes — bridge contract and agent tool surface move in lockstep. Default target world is the [`omnilink_omniarm6.omniworld`](../../../projects/samples/demos/worlds/chat/omnilink_omniarm6.omniworld) chat demo.

> **Note.** Axis previously lived in the OmniLink repo (at `agents/axis` there). It was migrated into OmniSim's `agents/production/` tree on 2026-05-13 so it ships next to the world and controller it drives. The OmniLink copy is kept as a forwarder pointing here.

## What Axis drives

The reference robot is the OmniArm 6 in OmniSim (robot id `omniarm6`), driven by the generic controller
`projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py` with `controllerArgs ["--robot" "omniarm6" ...]`. That controller defines the kinematic chain, joint limits, home pose, and damped-least-squares IK that Axis assumes by default:

- 6 revolute joints: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`
- `HOME_POSE = [-0.1732, 0.1855, 0.7417, 0.0, 2.2144, 0.0]`
- `IK_MAX_DQ = 0.08` rad per tick, `IK_DAMPING = 0.06`, `IK_TOL = 5e-3`, `IK_MAX_ITERS = 100`

Any other OmniSim robot exposed through the same bridge (mobile bases, grippers, sensors) is also in scope once its capabilities are registered.

## How Axis runs

Axis is continuously-on through OmniLink standing orders:

- **Telemetry tick** — every 1 s, reads joint state and TCP pose for every active robot.
- **Safety watchdog** — every 2 s, validates limits, faults, and telemetry freshness; issues `stop_robot` on any violation.
- **Session summary** — 18:00 daily, rolls up commanded setpoints and realized motion.

On top of the standing orders, operators and planners issue ad-hoc commands: `set_tcp_target`, `set_joint_positions`, `reset_to_home`, `stop_robot`, and so on.

## Integration shape

```
Operator / planner
      |
      v
OmniLink ──► Axis agent ──► OmniLink automation endpoints
                                    |
                                    v
                              OmniSim bridge
                                    |
                                    v
                        OmniSim controller (omnilink_arm_bridge.py --robot omniarm6)
                                    |
                                    v
                              Simulated OmniArm 6
```

Axis never imports the `controller` module. The OmniSim-side bridge owns all OmniSim API calls (`Robot()`, `getDevice`, `setPosition`, `robot.step`) and forwards a normalized HTTP surface to OmniLink.

## Operator expectations

- You always have an emergency `stop_robot` available and it is allowed for every authenticated session.
- Axis will refuse ambiguous motion commands ("a bit to the left") and ask you to confirm a concrete delta first.
- Axis will never claim a target was reached unless the next telemetry tick confirms it.
- Simulation topology changes (spawning objects, moving goals) are out of scope — Axis drives robots, not the world.

## Architecture stance

Axis follows the same productized-agent layout as OmniLink's first-party assistant agent:

- OmniLink profile scaffold in `profile.json`
- system prompt in `prompts/system.md`
- system design and contracts in `specs/`
- curated grounding docs in `knowledge/` (robot specs, bridge schema)
- agent-written durable notes in `long_term_memory/` (calibrations, operator overrides)
- auto-discovered tool modules in `tools/` (HTTP proxies to the OmniSim bridge + knowledge/memory tools)
- thin runner at `axis_agent.py`
- execution plan in `roadmap.md`

## Files

- [`profile.json`](profile.json) — OmniLink agent-profile scaffold, standing orders, commands/actions
- [`prompts/system.md`](prompts/system.md) — system prompt that feeds `mainTask`
- [`specs/architecture.md`](specs/architecture.md) — execution model, state, integration, safety
- [`specs/memory-model.md`](specs/memory-model.md) — robot-state, task, and safety memory tiers
- [`specs/tooling-contract.md`](specs/tooling-contract.md) — tool classes, action tiers, motion safety loop
- [`knowledge/`](knowledge/) — curated source-of-truth docs (robots, bridge, OmniLink)
- [`long_term_memory/`](long_term_memory/) — agent-written markdown notes, indexed with embeddings
- [`tools/`](tools/) — one file per tool group; auto-discovered by `axis_agent.py`
- [`axis_agent.py`](axis_agent.py) — thin runner: profile push + tool callback server
- [`roadmap.md`](roadmap.md) — phased execution plan
