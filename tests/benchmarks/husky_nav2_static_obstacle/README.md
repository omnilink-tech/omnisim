# Husky Nav2 static-obstacle benchmark

A manually invoked, single cold-start navigation experiment using the existing
Husky Tier-2 bridge and lean Nav2 stack. It checks scan-fed avoidance of one
static box, with simulator ground-truth odometry and static identity
`map -> odom`. It does not test AMCL, SLAM, localization accuracy, or hardware.

## Status

This migration is based on upstream commit
`5193feadadb98e5236c8738f162ea352919cfef8`, which includes ClockNode PR #25.
Recorded live validation on ThinThinkPad, WSL2 Ubuntu 24.04, Python 3.12.3,
ROS 2 Jazzy/CycloneDDS and CPU Newton/MuJoCo:

| Runner version | PASS | FAIL | INVALID |
|---|---:|---:|---:|
| Original runner | 1 | 1 | 0 |
| Startup diagnostics runner | 3 | 0 | 0 |

All five Nav2 actions SUCCEEDED and all five recorded measurement verdicts were
PASS. The original-runner FAIL was a pre-goal startup scan drop
in the Nav2 log. Contact positive control passed with three complete episodes.
The two groups have different startup observation conditions; the diagnostic
results do not establish a fix for the earlier scan drop or long-term stability.

A separate cold-start trial after the navigation pending-scan finalization and
DDS-metadata fixes also passed: action SUCCEEDED, navigation/measurement/overall
PASS, 15.808 simulation seconds and 33.962 monotonic wall seconds. Position error
was 0.145 m, yaw error 0.230 rad, traveled distance 4.876 m and minimum sampled
hull clearance 0.450 m, with zero observed obstacle contacts, sampled overlaps
or Nav2 scan drops. All 117 immediate scan TF failures were accounted for:
116 resolved and one startup-only expiry; none remained pending. Scan/TF DDS
metadata was populated. No retry straddled the action-result boundary in that
trial, so the exceptional finalization paths remain covered by synthetic tests.
This additional trial is separate from the two groups above.

The subsequent reuse of the same drain for contact-control finalization has
engine-free validation only; the positive-control run has not been repeated.

Historical v9.2.2 trials demonstrated physical detours and one validated
behavior/measurement PASS. Of three subsequent completed navigation trials,
only one passed all gates; the others retained scan-drop and/or TF-observer
failures. Earlier failed startup attempts were also retained. Those results do
not validate this baseline, and the ClockNode merge does not establish that
historical stale-scan incidents are resolved.

## Files and fixed scenario

- Runner: [`run_trial.py`](run_trial.py).
- World: [`husky_nav2_static_obstacle.omniworld`](../../../projects/samples/demos/worlds/chat/husky_nav2_static_obstacle.omniworld).
- Parameters: [`husky_static_obstacle_params.yaml`](../../../packages/omnisim-ros2/src/omnisim_ros2_nav2/params/husky_static_obstacle_params.yaml).
- Existing launch: [`omnisim_nav2_lean.launch.py`](../../../packages/omnisim-ros2/src/omnisim_ros2_nav2/launch/omnisim_nav2_lean.launch.py).

A static box, size `(0.6, 1.0, 1.2)` m at `(2, 0, 0.6)` m, has matching visual
and collision geometry. It blocks the nominal path from `(0, 0, yaw=0)` to
`(4, 0, yaw=0)` while leaving room on either side. The world uses a 16 ms basic
step. The conservative planar Husky collision envelope, including wheels,
extends to `x=±0.4937, y=±0.33465` m; it excludes visual-only bumpers.

The isolated YAML changes only `robot_radius` to `0.60` m and
`inflation_radius` to `0.75` m in both costmaps, relative to the original
parameters. Existing obstacle layers remain enabled. The local costmap stays
a 3 m rolling window; planner and controller configuration is unchanged.
The lean launch does not start collision_monitor. Its preserved 0.35 m stop
circle is inactive and provides no full-hull safety claim.

## Prerequisites and startup contract

Use Linux/WSL2 throughout, with ROS 2 Jazzy, `rmw_cyclonedds_cpp`, the existing
Nav2 dependencies, and a built OmniSim ROS workspace. See the
[ROS workspace guide](../../../packages/omnisim-ros2/README.md) and
[Linux source-build guide](../../../docs/developer/quickstart.md).
`xvfb-run`, a compatible built OmniSim engine and libController, the Newton
runtime, and rendering support are required. LiDAR requires rendering.

A Git worktree does not inherit ignored engine or ROS build/install outputs.
The validated environment resolves OmniSim Python, bridge Python, worlds and
controllers from this checkout, and sources Jazzy before this checkout's ROS
install overlay. The engine/libController and Newton runtime were compatible
local build products; matching runtime resources were supplied from the original
source build while retaining this worktree's source and ROS package resolution.
The loaded ClockNode implementation was checked for PR #25's retained steady
clock; Git HEAD alone is insufficient to establish runtime provenance.

A complete engine deployment requires generated `resources/proto-list.xml` and
`resources/projects/plugins/robot_windows/generic/libgeneric.so`, in addition to
the engine and controller libraries. These come from normal build/resource
deployment. A missing PROTO list caused a world-load HTTP 422 in the initial
worktree attempt. These local runtime products and ROS overlays are not PR files.
An old overlay must not silently select old ROS code.

Do not start the simulator, sidecar, or Nav2 separately. The runner starts its
own fresh processes and refuses occupied localhost ports 6789, 6790, and 8765.
It sets localhost proxy exclusions, `OMNISIM_URDF_USE_SENSORS=1`,
`OMNISIM_NEWTON_MODEL_DEVICE=cpu`, and `OMNISIM_NO_WINDOW=1`; it launches a
realtime engine with rendering enabled. Tier-2 rates are clock/state/odom/sensor
20/5/10/10 Hz, with root-only TF and LiDAR layer 0. It owns identity `map -> odom`
and launches the existing lean Nav2 stack with this experiment's parameters.

Both collector control and navigation use contact-only tracking (`light:false`,
grip/joint-limit tracking disabled). This adds measurement overhead relative
to the original light-mode demo. The runner sends zero velocity during cleanup
and terminates only its own process groups.

## Manual invocation

From this worktree's repository root, after preparing its engine and ROS
workspace:

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
source packages/omnisim-ros2/install/setup.bash
export OMNISIM_HOME="$PWD"
export OMNISIM_BINARY="$OMNISIM_HOME/bin/omnisim-bin"
export LD_LIBRARY_PATH="$OMNISIM_HOME/lib/webots${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$OMNISIM_HOME:$OMNISIM_HOME/packages/omnisim-bridges/src${PYTHONPATH:+:$PYTHONPATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
python tests/benchmarks/husky_nav2_static_obstacle/run_trial.py --check-geometry
python tests/benchmarks/husky_nav2_static_obstacle/run_trial.py --validate-collectors \
  --output /tmp/husky-nav2-obstacle-control
python tests/benchmarks/husky_nav2_static_obstacle/run_trial.py \
  --contact-control /tmp/husky-nav2-obstacle-control/result.json \
  --output /tmp/husky-nav2-obstacle-trial
```

These CLI forms and environment paths were exercised in the validated worktree;
navigation finalization and DDS metadata have one live PASS as described above.
The subsequent contact-control finalization change is engine-free tested only. Domain 0 must be
free of unrelated ROS publishers. Choose new output paths if those directories
already exist. Existing output is never overwritten. The collector control intentionally drives into the box and
backs away, without Nav2. A PASS control artifact with matched attributed
began/ended pairs is mandatory before navigation. Keep that artifact available
at the path passed to `--contact-control`.

## Acceptance and result classification

Before sending a goal, the runner requires TF-projected scan returns at the box
front and lethal global-costmap cells. During approach, it separately requires
local marking once the obstacle enters the local window, plus sampled evidence
of crossing the obstacle's x interval on a detour.

The existing behavior thresholds remain fixed:

| Metric | Requirement |
|---|---|
| Goal | Action succeeds without action error |
| Final position / yaw error | At most 0.25 m / 0.25 rad |
| Observed obstacle-contact episodes | Zero |
| Sampled hull overlaps | Zero |
| Minimum sampled hull clearance | At least 0.10 m |
| Sampled traveled distance | At most 8 m |
| Navigation duration | At most 60 simulation s and 120 monotonic wall s |
| Watchdog stops / detected stale-scan drop logs | Zero |

Measurement validity additionally requires scan/global/local marking evidence,
attributable and contiguous supervisor events with no unread loss, positive
contact-readback evidence, valid contact-control proof, and endpoint flatness
within 0.03 rad. The maximum merged trajectory gap is 0.20 simulation seconds.
A measurement-period scan TF retry expiry makes the result INVALID; the retry
budget remains two monotonic wall seconds per scan from its initial failure.
After the action result, no new scans enter this measurement observer. Pending
measurement-period scans are drained with TF subscriptions still alive; expiry
or unresolved finalization makes measurement INVALID without replacing the
navigation outcome. Startup-only failures are recorded separately.

The stale-scan gate scans the entire Nav2 process log, including startup before
the goal. A matching pre-goal drop therefore produces FAIL even if the action
later succeeds; startup diagnostic observer failures are distinct evidence.

`navigation_outcome` reports PASS or FAIL against behavior gates;
`measurement_outcome` reports PASS or INVALID against evidence requirements.
The combined `outcome` is INVALID whenever evidence is invalid, otherwise FAIL
if a behavior gate fails, otherwise PASS. Collector-only runs report navigation
as NOT_RUN. Process exit code is 0 only for combined PASS, otherwise 1.
A single PASS is a PoC result, not established repeatability.

## Evidence and limitations

The chosen output directory contains `result.json`, `samples.jsonl`,
`timing.jsonl`, `events.jsonl`, process logs (harness, engine, sidecar, static TF,
and Nav2 for navigation), and available engine sidecars. Results retain the
commit, machine metadata, preflight evidence, metrics, and failure reasons.
Samples retain scans/plans/costmap evidence and the metric pose samples; timing
records retain original bridge and native odometry channels and clock readings.
Event batches preserve unfiltered supervisor events, cursors, and receipt times.

`startup_timing.jsonl` contains compact scan stamps/frame IDs, relevant `/tf`
and `/tf_static` parent/child stamps, `/clock` values, monotonic and realtime
receipt timestamps, and observer/Nav2/goal/action boundaries. DDS metadata is
retained when supplied, otherwise null. Nav2 log boundaries retain their original
wall timestamps and are imported after cleanup. Diagnostics begin before Nav2,
exclude scan ranges, and never participate in scoring; diagnostic I/O errors are
reported to stderr. This independent subscriber cannot establish Nav2's internal
TF buffer contents or prove the cause of a message-filter drop.

The read-only 20 Hz bridge sampler and independent ROS receiver observe the same
simulator ground truth. Geometric metrics use their timestamp-sorted,
deduplicated union without interpolation. Native `/odom` publication/delivery
gaps remain separately reported; merged coverage is not evidence of gap-free
ROS publication. Wall durations use monotonic time.

Clearance is sampled Euclidean separation between a yaw-transformed conservative
robot rectangle and the known box. It is not costmap clearance, exact 3D
clearance, or a continuous-time guarantee. Traveled distance is a sampled lower
estimate. Zero observed contact episodes does not exclude contacts shorter than
supervisor observation intervals. Cumulative ring evictions are distinct from
unread event loss. Contact `t_sim_ms` uses the supervisor clock, not bridge engine
time.

The observer retries exact-stamp scan TF lookups asynchronously on a steady
clock; actual Nav2 message-filter drops are separate evidence. The current
stale-scan counter recognizes the existing log wording, not every possible
filter failure. The recorded diagnostic runs do not prove stale-scan resolution
or long-term reproducibility. Preserve historical FAIL/INVALID artifacts;
do not alter thresholds to turn incomplete evidence into PASS.
