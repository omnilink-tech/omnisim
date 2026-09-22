# OmniSim public beta

**Bring us something that misbehaves.** OmniSim is looking for its first
external users, and the most useful thing you can bring is a robot, controller
or world that does something you cannot explain. The useful outcome is not a
star: it is a completed install, a run that reproduced the problem, and an
honest report of where OmniSim's instruments helped, lied, or went quiet.

Research groups that want OmniLink to help port one bounded task, policy, or
robot should also read [OmniSim for research labs](LABS.md).

OmniSim is an Apache-2.0 robotics simulator built as an instrument for finding
out what a robot actually did. Run a controller and ask what happened: every
contact, every joint limit, every grip and every line the controller printed
arrive on one cursor-paged HTTP stream, alongside typed load diagnostics and
scene, joint and device inspection — all driveable by a coding agent, or by
`curl`. The repository also ships an MCP server, a ROS 2 sidecar, URDF import,
CPU and GPU Newton solvers, deformables, procedural world generation, and an
in-engine reinforcement-learning pipeline.

⚠️ Know the shape of the gap before you start: OmniSim **cannot control time
inside a run** — no pause, no breakpoints, no record/replay/diff, and a snapshot
restores poses rather than resuming a checkpoint. It observes and certifies; it
does not stop the clock.

## The 20-minute beta challenge

1. Install OmniSim:
   - **Windows 10/11:** use the asset on the
     [latest GitHub release](https://github.com/omnilink-tech/omnisim/releases/latest)
     (~600 MB). Install somewhere writable — under `C:\Program Files` the
     harness cannot write the sibling file it needs to load a world.
   - **Linux:** Ubuntu 24.04 or 22.04, `bash scripts/install/linux_bootstrap.sh`.
     Budget 25–45 minutes; most of it is the compile. Details in the
     [developer quickstart](docs/developer/quickstart.md). On 22.04 the
     bootstrap installs Python 3.12 (deadsnakes) for the engine to embed —
     the system 3.10 cannot run `newton`, and the build refuses to link it.
   - **macOS:** not supported. There is no package, no verified build, and
     Newton physics is unverified.
2. From the OmniSim directory, run `python -m omnisim doctor` and keep its
   output. It ends on a VERDICT line and exits non-zero if the install cannot
   run. On Windows without Python, `omnisim.bat doctor` uses the interpreter
   that ships in the package.
3. Run the real friction-grasp demo:

   ```bash
   python -m omnisim demo
   ```

4. Open the OmniSim directory you installed in step 1 in a coding agent — it
   ships `AGENTS.md`, and in Claude Code the checked-in `.mcp.json` registers
   the OmniSim MCP tools for you. Point it at the demo and ask it to
   investigate, not to build:

   ```text
   Load the OmniArm 6 real pick-and-place demo with {"light": false}, run it,
   and tell me every contact the gripper made, which joints hit their limits,
   and whether the grasp ever slipped. Quote the events, not a screenshot.
   ```

5. **Then bring your own misbehaving thing.** A URDF that falls over, a
   controller that works four times in five, a grasp that slips for no reason
   you can name — import it and try to make OmniSim tell you what happened. That
   is the run we most want a report about.

6. Tell us what happened. A failure report is at least as valuable as a
   success report, and "the instruments could not see it" is the single most
   useful sentence you can send us.

## What to report

Use the repository's issue forms:

- [Report a bug](https://github.com/omnilink-tech/omnisim/issues/new?template=bug_report.md)
- [Request a simulation](https://github.com/omnilink-tech/omnisim/issues/new?template=request_a_sim.yml)

Please include your operating system, `python -m omnisim doctor --fingerprint`
output (the `--fingerprint` half adds your OS, Python, GPU and physics versions,
which the plain report does not), the exact command or prompt you used, and the
first error message. Screenshots and
short screen recordings are welcome.

We especially want to learn:

- whether the installation instructions work on a clean machine;
- whether an unfamiliar coding agent can find and run a useful demo;
- which diagnostics are understandable without reading engine source;
- **whether OmniSim could explain a failure you brought it** — and, when it
  could not, exactly which question went unanswered;
- **whether any instrument gave you a confident answer that was wrong.** That is
  the highest-severity bug class in this project, above crashes;
- which robot, sensor, or physical phenomenon you expected but could not find;
- whether the HTTP/MCP workflow saves real iteration time for your project.

## Known limits

- **No time control inside a run**: no pause over HTTP, no breakpoints, no
  record/replay/diff. `POST /sim/snapshot` saves poses and joint angles only —
  never velocity, never solver state — so a restore is a teleport, not a
  resumed checkpoint.
- **Light mode is the default and silences 5 of the 10 event types**
  (`contact.*`, `grip.*`, `joint.limit_hit`). Load with `{"light": false}` for a
  debugging session. `GET /robot/<def>/sensor/<name>` is a deliberate 501:
  cameras, lidar and IMUs are read by their controller, not by the debug
  surface.
- **Fault injection is structural impact damage only** — no sensor dropout, no
  encoder drift, no actuator degradation, no latency or thermal models.
- The Windows package is the only downloadable desktop build currently
  certified by the project. Linux is source-build; macOS physics is unverified.
- ROS 2 support is new. Nav2 and MoveIt are not demonstrated.
- The renderer is real-time rasterization, not photorealistic RTX rendering.
- Sim-to-real transfer is unproven.
- OmniSim's G1 walk uses a weight-bearing balance harness; it is not a
  free-standing humanoid walk.
- The measured capability matrix currently reports 95.7% working coverage
  (51 probes, 2026-09-01) and names every absent, degraded, or broken probe.

The longer, claim-by-claim limitations list is in
[README.md](README.md#what-omnisim-is-worse-at). OmniSim does not collect beta
telemetry. Feedback reaches the project only when you choose to post it.
