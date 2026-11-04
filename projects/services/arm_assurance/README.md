# OmniArm 6 command-assurance pilot

A bounded, no-payload arm workcell in OmniSim. It accepts four named joint
poses (`home`, `inspect`, `transfer`, `park`) in the operator's requested order,
or explicit numeric joint targets inside the configured joint cell. A step
counts only after measured joint arrival and rest. Accepted work is not a
completed task.

This pilot tests useful motion, questions, quotations, negations, persistent
holds, explicit resume, targets outside the workcell, cancellation and Stop.
It includes an independent offline audit of joint traces and verified receipts,
controlled bad tool proposals and useful-motion controls. It does not establish
an unseen-command error rate or universal safety/hallucination immunity.

## Requirements

Use a writable public OmniSim 9.2.7 checkout with its engine, Newton runtime,
public OmniLink SDK and dependencies installed. Windows has the only prebuilt
package; other platforms require a source build. The window-state integration
regression has been verified on Windows. Connect your own OmniKey and active
model provider through the [normal setup](../../../docs/guide/omnilink-chat-demos.md).
AI usage is separate. No credentials, account or credits are bundled.

```powershell
python -m omnisim doctor
# Proceed only with VERDICT READY. Set OMNI_KEY privately in this environment.
$env:OMNILINK_ENGINE = 'g1-engine'
$env:OMNILINK_MODEL = ''
python arm_assurance.py run --episode 0 --out .tmp/arm-live-new
```

`run` defaults to one episode, containing eleven live requests. `suite` runs
twenty fresh episodes (220 live requests) and consumes provider usage. All
outputs require new directories. Each run owns and cleans up its own simulator
and controllers. No other simulator is closed.

Each run has a fresh local relay/history and private intent/action journal,
with remote memory and profile synchronization disabled. Account-side scratch
profile names are reused sequentially, rather than registering a new profile
for every run. This avoids consuming the account's agent-profile allowance.
It does not make the requests independent samples of unseen language.

For a single run, `OMNILINK_AGENT_TAG` selects an existing scratch profile
suffix; the default is the stable `arm-assurance-demo`. Parallel suites use
one distinct stable identity per slot (`arm-assurance-slot-01`, etc.). Set
`OMNISIM_ARM_AGENT_TAGS` to a comma-separated list of existing scratch suffixes
to reuse them instead. Never use a production robot's identity. Keep separate
identities for concurrent suite slots and a concurrent capture/control run.
The manifest records the selected tag and the isolation settings.

```powershell
python arm_assurance.py suite --parallel 1 --out .tmp/arm-suite-new
python arm_assurance.py controlled --key-file YOUR_PRIVATE_KEY_FILE --out .tmp/arm-controlled-new
python arm_assurance.py verify .tmp/arm-live-new
```

The optional `--key-file` reads a private OmniKey file into process memory.
Do not put credentials in the project or send a provider key to the robot.
An absent key or provider error is an error; there is no local AI fallback.
The independent typed Stop control is available separately.

## Scope and evidence

- Fixed-base OmniArm 6 URDF, six joint position sensors, no gripper or payload.
- Named poses and a narrow configured joint cell, not the robot's full reach.
- Newton 1.5 / MuJoCo CPU, 16 ms, eight configured substeps, empty floor.
- No camera perception, obstacle model, collision avoidance, force control,
  manipulation or physical-robot validation. Visual pads are markers only.
- Holds, explicit halt controls and leading Resume/Unlock requests are
  deterministic controls in this profile;
  their success is not proof that a model understood every safety instruction.
- Public outcome text reports verified arm state. Raw model replies are retained
  separately; grounded outcome reporting is not a general hallucination score.

The manifest pins source, binary/runtime and artifact hashes, records errors
and cleanup, and includes the Newton finalization sidecar and machine
fingerprint. Never discard FAIL or ERROR attempts. A source change requires a
new frozen campaign. To audit a historical campaign, use its archived verifier
and sources, rather than silently applying a later grading version.

Development findings and the first interrupted campaign are retained under
`tests/assessment/evidence/arm-assurance-20261030`. Qualification results must
be read with their campaign's report; the planned 220 requests are not a claim
that all 220 have completed or passed.
