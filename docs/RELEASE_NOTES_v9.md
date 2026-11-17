# OmniSim v9 — release preview

**A robotics workshop with connected OmniLink agents.**

This is an unreleased preview. The checkout still reports 8.5.1; a v9 package
and public snapshot must pass release validation before publication.

## Featured experience

[Watch OmniLink drive a Husky](media/omnilink-husky/omnilink-husky-live-ai.mp4):
one robot, one arena, instructions, follow-up questions and a return to base.
The 100-second film uses live model calls and native OmniSim footage. Waiting
is removed and motion is time-compressed. [Recording notes](media/omnilink-husky/README.md).

OmniLink requires an OmniKey and a connected model provider on every plan,
including Free. Model usage is billed separately. Missing credentials and
connection failures return errors; there is no keyless chat fallback.
Direct simulator controls and Stop remain independent.
[Connect your account and launch the Husky](guide/omnilink-chat-demos.md).

## Integration changes

- Windows bundling and Linux bootstrap include a shared public OmniLink
  dependency lock. `doctor --omnilink` checks the controller interpreter;
  Windows launchers prefer the verified bundle over system Python.
- Shared robot integration lives in `packages/omnisim-bridges`; worlds,
  controllers and robot windows remain under `projects/`.
- Connected chat spans mobile, arm, quadruped and drone interfaces. A successful
  Husky recording is not proof of equivalent task performance on every robot.
- Robot command results and live state support checking what actually happened.
  A model's acknowledgement alone does not establish successful motion.
- Access checks reject disconnected prompts before entering robot commands.
  Independent Stop remains available.

The simulator integration is source-shipped. This release does not distribute
the private OmniLink service or make published client code confidential.

## Supporting simulation studies

- [SO101 pick and place](../sim-to-real/so101/README.md): real footage beside an
  authored simulation that succeeds; exact recorded replay remains unresolved.
- [ALOHA battery insertion](../sim-to-real/aloha-battery/README.md): spring
  compression and fingertip seating in a mechanical reconstruction. Geometry
  and mechanism parameters are estimates; recorded actions fail to reproduce insertion.

Both studies retain videos, source attribution, controls and evidence.
**Neither establishes validated transfer to hardware.** The hardware bridge
examples use mock drivers; no learned policy is validated on a physical robot.

## Release checks still required

Build and verify the final installer, including OmniLink SDK dependencies in
the interpreter that actually runs robot controllers. Validate connection,
disconnection and Stop from that installed package without a developer PYTHONPATH.
Then run the repository's release checks and prepare its versioned public snapshot.
The checked-in demo assets and local source changes alone are not a completed release.

Earlier parser-only cost and accuracy experiments are not measurements of the
live-AI film or the current connected product. No zero-model-cost, universal
accuracy, market exclusivity or hardware-performance claim is made here.
