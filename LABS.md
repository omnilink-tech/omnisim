# OmniSim for research labs

OmniSim is looking for research groups that want to test a policy, task, robot,
or benchmark in an independent simulator. During the public beta, OmniLink is
offering hands-on engineering support for a small number of sharply scoped
pilots.

The useful outcome is not a logo, a sponsorship announcement, or a screenshot.
It is a reproducible result: one task or policy running with exact commands,
measured success and failure criteria, and a written account of every semantic
gap we found.

## What OmniLink can offer

For an accepted pilot, we can do the OmniSim-side work:

- **Policy portability:** map joint ordering, observations, actions, timing,
  resets, contacts, and terrain for one released policy or checkpoint.
- **Benchmark portability:** reproduce one bounded task while preserving its
  observation, action, reset, success, and partial-progress contracts.
- **Robot bring-up:** import a legally redistributable URDF or STEP model,
  verify orientation and collision geometry, and document the resulting model.
- **Contact and manipulation validation:** test a grasp, tactile observation,
  or force-sensing contract against physical outcomes and negative controls.
- **Agent integration:** expose a live scene through HTTP/JSON or the OmniSim
  MCP server so a coding agent can load, inspect, edit, step, and verify it.
- **Synthetic data:** generate aligned RGB, depth, instance segmentation, and
  camera metadata for an agreed offline scene and randomization protocol.
- **Reproducibility work:** provide exact launch commands, environment details,
  seeds, machine attribution, logs, and a result report that includes failures.

We keep the initial work on the OmniSim side. The lab does not need to maintain
an OmniSim backend or accept a pull request. If the experiment becomes useful,
we can discuss the smallest appropriate integration afterward.

## Good first pilots

A good first pilot fits in one sentence and has a measurable boundary. Examples:

- replay one released G1 or H1 motion-tracking policy and report tracking error,
  falls, and interface mismatches;
- reproduce one pick-and-place task and preserve its Gymnasium-facing contract;
- port one Franka tactile task and validate binary contact observations;
- express one symbolic household task and its partial-success predicates;
- bring up one lab robot from URDF and let an agent create and verify a test
  world around it.

If the first request is "port our entire benchmark," we will help reduce it to
the smallest experiment that can honestly answer a useful question.

## What OmniSim provides today

- Apache-2.0 source code, with a downloadable Windows beta package and a
  verified Linux source build.
- CPU MuJoCo and GPU MuJoCo-Warp solver paths through Newton.
- URDF import and a STEP-to-URDF conversion workflow.
- An agent-facing HTTP/JSON harness, a first-party MCP server, capture tooling,
  and a ROS 2 sidecar.
- Rigid-body, cloth, soft-body, manipulation, procedural-world, and in-engine
  reinforcement-learning workflows.
- A committed capability matrix and benchmark reports that name the machine,
  method, limitations, and negative results behind each claim.

Start with the [20-minute beta challenge](BETA.md), the
[demo catalogue](DEMOS.md), or the [agent entry point](AGENTS.md).

## Limits we will state up front

OmniSim is not photorealistic. Physical-robot transfer has not been established.
Its in-house G1 walking result is weight-bearing rather than free-standing.
ROS 2 integration is new, macOS physics is unverified, and the measured
capability matrix includes broken and degraded features. The public
[limitations list](README.md#what-omnisim-is-worse-at) is part of every pilot,
not fine print.

## What we need from a lab

The lab chooses one canonical task, policy, checkpoint, or robot and identifies
the result it considers meaningful. It also confirms that any supplied assets,
weights, and datasets may be used for the proposed experiment. We handle the
OmniSim implementation and return a reproducible result for technical review.

## Contact

Email **[info@omnilink-agents.com](mailto:info@omnilink-agents.com)** with:

1. the repository or paper;
2. the smallest task, policy, or robot you want tested;
3. your required observation/action/success contract; and
4. the hardware and operating system you expect to use.

GitHub: [omnilink-tech/omnisim](https://github.com/omnilink-tech/omnisim)


