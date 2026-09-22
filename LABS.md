# OmniSim for research labs

**Bring us the result you cannot explain.** OmniSim is looking for research
groups with a policy, task, robot or benchmark that behaves in a way they have
not been able to account for — a policy that transfers to one simulator and not
another, a grasp that succeeds four times in five, a benchmark whose failures
have no diagnosis. During the public beta, OmniLink is offering hands-on
engineering support for a small number of sharply scoped pilots: we run it in
OmniSim and report what the instruments saw.

An independent second simulator is useful on its own, and that offer stands. But
the pilots worth your time are the ones where the answer is currently a guess.

The useful outcome is not a logo, a sponsorship announcement, or a screenshot.
It is a reproducible result: one task or policy running with exact commands,
measured success and failure criteria, and a written account of every semantic
gap we found — including the questions OmniSim could not answer.

## What OmniLink can offer

For an accepted pilot, we can do the OmniSim-side work:

- **Failure diagnosis:** reproduce one misbehaviour in OmniSim and account for
  it from the instruments — the contact set, joint-limit events, grips, device
  state and controller logs on one cursor-paged stream — and say plainly when
  the instruments cannot settle the question.
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

- take one intermittent failure — a grasp that slips, a gait that trips on a
  particular tile — reproduce it in OmniSim, and report what the contact,
  joint-limit and grip records say was happening at the moment it went wrong;
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
- A debugging surface: 10 typed runtime event types with drop counters, 50+
  structured load diagnostics, and contact, joint (`hit_limit`), grip, device
  and bounds inspection — each read served from one consistent instant, and each
  one declaring its own completeness rather than returning a bare empty list.
- Bitwise run-to-run reproduction on the CPU MuJoCo solver (the default),
  measured on contact-rich ten-robot scenes across cold launches and across two
  machines running the same binary. On the GPU `mujoco_warp` solver
  reproduction is **refuted, not unmeasured** — 0 of 24 same-config pairs.
- Rigid-body, cloth, soft-body, manipulation, procedural-world, and in-engine
  reinforcement-learning workflows.
- A committed capability matrix and benchmark reports that name the machine,
  method, limitations, and negative results behind each claim.

Start with the [20-minute beta challenge](BETA.md), the
[demo catalogue](DEMOS.md), or the [agent entry point](AGENTS.md).

## Limits we will state up front

OmniSim observes and certifies; it does not control time inside a run. There is
no pause over HTTP, no breakpoints, and no record/replay/diff, and
`POST /sim/snapshot` saves poses and joint angles only — never velocity, never
solver state — so a restore is a teleport rather than a resumed checkpoint. Its
fault injection is structural impact damage, not sensor dropout, encoder drift,
actuator degradation, latency or thermal derating. Sensor samples are not
readable through the debug surface: `GET /robot/<def>/sensor/<name>` is a
deliberate 501, and cameras, lidar and IMUs are read by their own controller.

OmniSim is not photorealistic. Physical-robot transfer has not been established.
Its in-house G1 walking result is weight-bearing rather than free-standing.
ROS 2 integration is new, macOS physics is unverified, and the measured
capability matrix includes broken and degraded features. The public
[limitations list](README.md#what-omnisim-is-worse-at) is part of every pilot,
not fine print.

## What we need from a lab

The lab chooses one canonical task, policy, checkpoint, or robot — ideally one
whose behaviour it cannot currently account for — and identifies the result it
considers meaningful. It also confirms that any supplied assets, weights, and
datasets may be used for the proposed experiment. We handle the OmniSim
implementation and return a reproducible result for technical review, including
the parts we could not explain.

## Contact

Email **[info@omnilink-agents.com](mailto:info@omnilink-agents.com)** with:

1. the repository or paper;
2. the smallest task, policy, or robot you want tested — and, if there is one,
   the behaviour you cannot explain;
3. your required observation/action/success contract; and
4. the hardware and operating system you expect to use.

GitHub: [omnilink-tech/omnisim](https://github.com/omnilink-tech/omnisim)

