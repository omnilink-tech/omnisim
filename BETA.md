# OmniSim public beta

OmniSim is looking for its first external users. The useful outcome is not a
star: it is a completed install, a simulation that ran, and an honest report of
where the experience became confusing or broke.

Research groups that want OmniLink to help port one bounded task, policy, or
robot should also read [OmniSim for research labs](LABS.md).

OmniSim is an Apache-2.0 robotics simulator built for coding agents. An agent
can load a world, inspect the scene, step physics, capture an image, and hot
reload changes over HTTP/JSON. The repository also ships an MCP server, a ROS 2
sidecar, URDF import, CPU and GPU Newton solvers, deformables, procedural world
generation, and an in-engine reinforcement-learning pipeline.

## The 20-minute beta challenge

1. Install OmniSim:
   - **Windows 10/11:** use the asset on the
     [latest GitHub release](https://github.com/omnilink-tech/omnisim/releases/latest).
   - **Linux:** follow the
     [developer quickstart](docs/developer/quickstart.md). The source build is
     verified; a general-purpose Linux binary is not published yet.
   - **macOS:** not part of this beta. Newton physics on macOS is unverified.
2. From the OmniSim directory, run `python -m omnisim doctor` and keep its
   output.
3. Run the real friction-grasp demo:

   ```bash
   python -m omnisim run-world projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld
   ```

4. Open the repository in a coding agent that reads `AGENTS.md` and ask:

   ```text
   Add a blue cylinder beside the place table in the OmniArm 6 real
   pick-and-place demo, run a load check, and show me a screenshot.
   ```

5. Tell us what happened. A failure report is at least as valuable as a
   success report.

## What to report

Use the repository's issue forms:

- [Report a bug](https://github.com/omnilink-tech/omnisim/issues/new?template=bug_report.md)
- [Request a simulation](https://github.com/omnilink-tech/omnisim/issues/new?template=request_a_sim.yml)

Please include your operating system, `python -m omnisim doctor` output, the
exact command or prompt you used, and the first error message. Screenshots and
short screen recordings are welcome.

We especially want to learn:

- whether the installation instructions work on a clean machine;
- whether an unfamiliar coding agent can find and run a useful demo;
- which diagnostics are understandable without reading engine source;
- which robot, sensor, or physical phenomenon you expected but could not find;
- whether the HTTP/MCP workflow saves real iteration time for your project.

## Known limits

- The Windows package is the only downloadable desktop build currently
  certified by the project. Linux is source-build; macOS physics is unverified.
- ROS 2 support is new. Nav2 and MoveIt are not demonstrated.
- The renderer is real-time rasterization, not photorealistic RTX rendering.
- Sim-to-real transfer is unproven.
- OmniSim's G1 walk uses a weight-bearing balance harness; it is not a
  free-standing humanoid walk.
- The measured capability matrix currently reports 78% working coverage and
  names every absent, degraded, or broken probe.

The longer, claim-by-claim limitations list is in
[README.md](README.md#what-omnisim-is-worse-at). OmniSim does not collect beta
telemetry. Feedback reaches the project only when you choose to post it.
