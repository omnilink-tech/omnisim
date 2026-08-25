# Webots bridge — exclusions from the wrapped function surface

**Contract.** The Phase W `shell+tools` condition wraps *the entire published
Supervisor and Robot function reference of upstream Webots R2025a*
([agent-edge-validation-plan.md](../../../../../docs/developer/agent-edge-validation-plan.md)
§2.2). A bridge faithful in every included verb but curated by omission is the
abuse the fidelity rules alone cannot catch, so **every public function of the
local controller package's `Robot`, `Supervisor`, `Node`, `Field`, `Proto`,
`Device` and `ContactPoint` classes that is not wrapped must appear here with
a justification** — enforced mechanically by
`runner/tools/test_webots_bridge.py` (the completeness test), and
countersigned by the §6.2.4 non-OmniSim reviewer at pre-registration. When in
doubt, the rule was to wrap it; everything below is excluded for a structural
reason, not a convenience one.

Enumeration source: `lib/controller/python/controller/` (this repository is a
Webots fork; its controller package is upstream R2025a's Supervisor/Robot API
surface). A final cross-check against cyberbotics.com's published R2025a
function index is a V5 review-window item.

Format note: the completeness test recognises an exclusion by its
`` `Class.method` `` token; keep one token per row.

---

## 1. Deprecated Python-only device accessors (not in upstream's function index)

These 24 methods are Python-API legacy aliases of `getDevice`. Upstream's
R2025a C function index has no corresponding `wb_robot_get_accelerometer`
etc., and calling any of them prints a deprecation warning redirecting to
getDevice — which **is** wrapped (`wb_robot_get_device`). Wrapping
them would add 24 redundant tools that upstream's own reference does not
publish, biasing the tool-count comparison for no capability.

| excluded | reason |
|---|---|
| `Robot.getAccelerometer` | deprecated alias of `getDevice` (wrapped as `wb_robot_get_device`) |
| `Robot.getAltimeter` | deprecated alias of `getDevice` |
| `Robot.getBrake` | deprecated alias of `getDevice` |
| `Robot.getCamera` | deprecated alias of `getDevice` |
| `Robot.getCompass` | deprecated alias of `getDevice` |
| `Robot.getConnector` | deprecated alias of `getDevice` |
| `Robot.getDisplay` | deprecated alias of `getDevice` |
| `Robot.getDistanceSensor` | deprecated alias of `getDevice` |
| `Robot.getEmitter` | deprecated alias of `getDevice` |
| `Robot.getGPS` | deprecated alias of `getDevice` |
| `Robot.getGyro` | deprecated alias of `getDevice` |
| `Robot.getInertialUnit` | deprecated alias of `getDevice` |
| `Robot.getLED` | deprecated alias of `getDevice` |
| `Robot.getLidar` | deprecated alias of `getDevice` |
| `Robot.getLightSensor` | deprecated alias of `getDevice` |
| `Robot.getMotor` | deprecated alias of `getDevice` |
| `Robot.getPen` | deprecated alias of `getDevice` |
| `Robot.getPositionSensor` | deprecated alias of `getDevice` |
| `Robot.getRadar` | deprecated alias of `getDevice` |
| `Robot.getRangeFinder` | deprecated alias of `getDevice` |
| `Robot.getReceiver` | deprecated alias of `getDevice` |
| `Robot.getSkin` | deprecated alias of `getDevice` |
| `Robot.getSpeaker` | deprecated alias of `getDevice` |
| `Robot.getTouchSensor` | deprecated alias of `getDevice` |

## 2. Human-input channels (structurally zero by SPEC §3.4)

AgentBench has **no human channel**: no TTY, no GUI the operator can reach,
`interventions` is zero by construction. Keyboard, mouse and joystick input
can therefore never arrive, and a tool that blocks waiting for it can only
burn the run's clock. These are also documented by upstream as separate
device APIs (`wb_keyboard_*`, `wb_mouse_*`, `wb_joystick_*`), outside the
Supervisor + Robot (device-generic) scope this bridge wraps.

| excluded | reason |
|---|---|
| `Robot.getKeyboard` | returns the keyboard human-input device; no human channel exists in a scored run |
| `Robot.getMouse` | returns the mouse human-input device; same |
| `Robot.getJoystick` | returns the joystick human-input device; same |
| `Robot.waitForUserInputEvent` | blocks up to `timeout` for keyboard/mouse/joystick events that structurally cannot occur; would stall the bridge loop for nothing |

## 3. Robot-window (wwi) messaging

The robot-window HTML/JS plugin does not exist in the benchmark container
(headless, no GUI, no browser); `wwiSendText` writes to, and
`wwiReceiveText` reads from, a window that is never created. Named as an
expected exclusion by the task contract.

| excluded | reason |
|---|---|
| `Robot.wwiSendText` | sends to the robot-window plugin; no robot window exists headless |
| `Robot.wwiReceiveText` | receives from the robot-window plugin; same |

## 4. Movie recording and animation export

Named as expected legitimate exclusions. Both produce artifacts (an encoded
movie of the 3D view; an HTML5 animation) that no grader reads and no task
requires, both depend on the GUI rendering path of the benchmark's headless
invocation, and movie encoding additionally depends on codecs the pinned
container does not ship. Still-image capture **is** wrapped
(`wb_supervisor_export_image`), so visual evidence is not curated away.

| excluded | reason |
|---|---|
| `Supervisor.movieStartRecording` | encodes a movie of the GUI 3D view; headless container, no codec stack, no grader consumes it |
| `Supervisor.movieStopRecording` | pair of the above |
| `Supervisor.movieIsReady` | status query for the above |
| `Supervisor.movieFailed` | status query for the above |
| `Supervisor.animationStartRecording` | exports an HTML5 animation for a browser; meaningless in the benchmark container |
| `Supervisor.animationStopRecording` | pair of the above |

## 5. Virtual-reality headset

There is no VR headset in a benchmark container; upstream's own docs scope
these to a physically attached HMD. `virtualRealityHeadsetIsUsed` would
truthfully return false and the two getters return garbage when no headset
is used (per upstream's reference).

| excluded | reason |
|---|---|
| `Supervisor.virtualRealityHeadsetIsUsed` | no HMD hardware in the container |
| `Supervisor.virtualRealityHeadsetGetPosition` | undefined without an HMD |
| `Supervisor.virtualRealityHeadsetGetOrientation` | undefined without an HMD |

## 6. Data-carrier accessors (not API functions)

`ContactPoint` is a value object returned by
`wb_supervisor_node_get_contact_points` (which **is** wrapped — task B1
depends on it). The bridge serialises every contact point inline as
`{point, node_id, depth}`, so its Python accessors have nothing left to
fetch across the wire. (`depth` is a fork addition to the value object; it
is included in the serialisation and noted for the reviewer, but it is not a
function surface.)

| excluded | reason |
|---|---|
| `ContactPoint.getPoint` | serialised inline as `point` |
| `ContactPoint.getNodeId` | serialised inline as `node_id` |
| `ContactPoint.getDepth` | serialised inline as `depth` |

## 7. Python property aliases of wrapped functions

The Python API mirrors most getter/setter pairs as properties. Each row
below is byte-for-byte the same engine call as the wrapped function named
beside it; upstream's function index documents the function form, and a
JSON-RPC bridge has no property syntax to offer. Nothing is reachable
through a property that is not reachable through a wrapped tool.

| excluded property | same call as |
|---|---|
| `Robot.basic_time_step` | `wb_robot_get_basic_time_step` |
| `Robot.name` | `wb_robot_get_name` |
| `Robot.model` | `wb_robot_get_model` |
| `Robot.custom_data` | `wb_robot_get_custom_data` / `wb_robot_set_custom_data` |
| `Robot.project_path` | `wb_robot_get_project_path` |
| `Robot.world_path` | `wb_robot_get_world_path` |
| `Robot.supervisor` | `wb_robot_get_supervisor` |
| `Robot.synchronization` | `wb_robot_get_synchronization` |
| `Robot.time` | `wb_robot_get_time` |
| `Robot.number_of_devices` | `wb_robot_get_number_of_devices` |
| `Robot.battery_sensor_sampling_period` | `wb_robot_battery_sensor_get_sampling_period` / `wb_robot_battery_sensor_enable` |
| `Robot.mode` | `wb_robot_get_mode` |
| `Supervisor.simulation_mode` | `wb_supervisor_simulation_get_mode` / `wb_supervisor_simulation_set_mode` |
| `Node.DEF` | `wb_supervisor_node_get_def` |
| `Node.id` | `wb_supervisor_node_get_id` |
| `Node.type` | `wb_supervisor_node_get_type` |
| `Node.type_name` | `wb_supervisor_node_get_type_name` |
| `Node.base_type_name` | `wb_supervisor_node_get_base_type_name` |
| `Node.number_of_fields` | `wb_supervisor_node_get_number_of_fields` |
| `Node.number_of_base_node_fields` | `wb_supervisor_node_get_number_of_base_node_fields` |
| `Field.name` | `wb_supervisor_field_get_name` |
| `Field.type_name` | `wb_supervisor_field_get_type_name` |
| `Field.count` | `wb_supervisor_field_get_count` |
| `Field.value` | the typed `wb_supervisor_field_get_sf_*` / `set_sf_*` family |
| `Proto.type_name` | `wb_supervisor_proto_get_type_name` |
| `Proto.number_of_fields` | `wb_supervisor_proto_get_number_of_fields` |
| `Proto.is_derived` | `wb_supervisor_proto_is_derived` |
| `Device.name` | `wb_device_get_name` |
| `Device.model` | `wb_device_get_model` |
| `Device.node_type` | `wb_device_get_node_type` |

## 8. Structural (not enumerated as API surface)

Constructors (`__init__`), destructors (`__del__`), class constants
(`Node.ROBOT`, `Field.SF_BOOL`, `Robot.MODE_*`, ...), and the singleton
guard attribute `Robot.created` are not functions of the published
reference; the completeness test's enumerator skips underscore names and
non-function attributes by construction. The bridge owns the one
`Supervisor()` instance per controller process (upstream allows exactly one
`Robot` instance per controller), so object construction is not a wire verb.
