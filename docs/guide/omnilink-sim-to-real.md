# From simulation to a hardware control surface

OmniSim and a hardware adapter can expose the same robot tools and HTTP
contract. What carries over is the application above the driver: tool names,
arguments and results. A physical driver must implement those operations and
report what actually happened.

**This is not validated transfer to hardware.** The examples in
[`agents/bridges/`](../../agents/bridges/) use mock drivers. No learned policy
trained here has been validated on a physical robot. Dynamics, sensing, latency
and actuator response still require hardware-specific implementation and checks.

## Watch the comparison studies

- [SO101 pick and place](../../sim-to-real/so101/README.md): authored simulation
  succeeds beside a real recording; exact recorded replay remains unresolved.
- [ALOHA battery insertion](../../sim-to-real/aloha-battery/README.md): a spring
  mechanism reconstruction with authored motion. Estimated parameters and
  unsuccessful recorded-action replay are documented alongside the video.

These studies retain source attribution, controls and measurements. They do
not show OmniLink driving physical hardware or establish a hardware success rate.

## Integration locations

| Responsibility | Location |
|---|---|
| Reusable robot bridge interfaces | [`packages/omnisim-bridges/`](../../packages/omnisim-bridges/) |
| Simulator robot controllers | [`projects/samples/demos/controllers/`](../../projects/samples/demos/controllers/) |
| Runnable connected-agent worlds | [`projects/samples/demos/worlds/chat/`](../../projects/samples/demos/worlds/chat/) |
| Mock hardware adapters | [`agents/bridges/`](../../agents/bridges/) |
| OmniKey and model setup | [Connected robot guide](omnilink-chat-demos.md) |
| Interface limits | [Actuation coverage](../../packages/omnisim-bridges/GATE_COVERAGE.md) |

Keep new robot-specific driver code beside its robot project, and reusable
interface code in the bridge package. Avoid copying shared implementation into
each demo. The private OmniLink service belongs in the OmniLink project; the
client integration shipped here is inspectable source.

## Port one bounded task

Start from the robot's supported typed operations and implement the driver for
each one. Return measured outcomes where possible, and report unsupported
operations explicitly. Connect an authenticated OmniLink relay to enable chat;
the mock examples' `/prompt` does not provide a keyless agent.

Test the driver's independent Stop operation and each primitive before a
multi-step task. A simulator reset can teleport a model; do not implement a
physical reset as a teleport or silently equate it with homing. Do not infer
collision avoidance or a safety certification from the shared interface.

Only claim hardware performance after testing that hardware and recording the
conditions and outcome. [v9 release scope](../RELEASE_NOTES_v9.md).
