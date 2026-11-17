# Axis Roadmap

## Phase 1: Identity and contracts

- define Axis's role, boundaries, and operator expectations as a robot-control agent
- create the OmniLink profile scaffold with telemetry/watchdog/session-summary standing orders
- define prompt, memory, tooling, and bridge contracts

## Phase 2: Runtime integration

- add a seeded Axis profile path in the OmniLink UI
- load Axis defaults into agent profile creation/edit flows
- support exporting or syncing `agents/axis/profile.json` into platform state

## Phase 3: OmniSim bridge

- stand up the OmniSim bridge as a separate service wrapping `omnilink_arm_bridge.py --robot omniarm6` (reference controller)
- publish the normalized command surface (`list_robots`, `set_tcp_target`, `stop_robot`, ...)
- implement bridge-side joint-limit clamping and standardized fault codes
- add robot capability registration so new robots show up in `list_robots` without runner changes

## Phase 4: Tool-side proxies and safety loop

- ship the `tools/robots.py` HTTP-proxy tools (present)
- implement the observe → validate → step-bound → command → confirm → log motion loop in the system prompt
- wire the 1 s telemetry tick and 2 s safety watchdog into OmniLink's standing-orders runtime
- cap joint steps at `IK_MAX_DQ` client-side and server-side; surface violations as alerts

## Phase 5: Trajectory planning

- implement `plan_trajectory` on the bridge (time-parameterized paths with velocity/acceleration/blend constraints)
- enable `execute_trajectory` to stream planned setpoints through the bridge clamp on every tick
- add reusable trajectory recipes — persist successful paths to long-term memory as templates
- flag repeated manual overrides for planner review

## Phase 6: Fleet and multi-robot

- scale from one OmniArm 6 to heterogeneous robots (different joint counts, mobile bases, grippers, sensors)
- add per-robot scope checks (`robot.control`, `robot.read`, `sim.admin`)
- introduce the tiered-manifest path for when capability records push the tool count past the full-manifest threshold

## Phase 7: Perception and coordination

- surface sensor reads (`read_sensor`) for force/torque and camera loops
- hand scene-topology changes to a separate scene-control agent and coordinate handoffs
- explore closed-loop visual servoing once the bridge exposes a camera pipeline

## Near-term next step

Stand up the OmniSim bridge as a stub that serves `list_robots` + `get_robot_state` against a static capability record, then wire the Axis runner end-to-end: `axis_agent.py` registers the profile, operator asks `get_robot_state('omniarm6_01')`, response comes back through the tool-callback server. Motion commands follow once the bridge has real OmniSim control.
