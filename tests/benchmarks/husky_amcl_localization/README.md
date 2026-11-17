# Husky AMCL measured localization regression

This manually invoked benchmark loads the known-wall Husky world and its
deterministic map, starts stock Jazzy localization, and measures initial and
post-motion stationary localization errors against independent harness poses.
It commands approximately 0.4 m forward and a 0.35 rad turn, then stops and
cleans up only its owned processes. It runs no navigation, obstacle benchmark
or SLAM.

AMCL receives **simulator ground-truth odometry** as its motion input and starts
with the known origin seed. This experiment does not establish localization
robustness under realistic wheel odometry drift, or convergence from an uncertain
initial pose.

Benchmark B also records fresh error samples in an initial and post-motion
stationary observation window, measures stable convergence and localization
jitter, and provides a sequential independent cold-start repeat entry point.

## Run one trial

Requires Linux, Python 3.12, ROS 2 Jazzy with Nav2 and CycloneDDS, a compatible
engine/controller deployment in this checkout, CPU Newton physics packages,
Qt/wgpu renderer dependencies and a working display (WSLg or Xvfb). Build the
scoped `omnisim_ros2` overlay beforehand. The runner never installs or builds
dependencies. Rendering remains enabled for LiDAR even in no-window mode.

From the repository root:

```bash
python3 -B tests/benchmarks/husky_amcl_localization/run_trial.py
```

For three independent cold starts, each with its own process lifecycle and output:

```bash
python3 -B tests/benchmarks/husky_amcl_localization/run_repeated.py --runs 3 --output /tmp/husky-amcl-repeat
```

The repeat directory must be new. Other trial options are forwarded unchanged,
for example `--no-score` or `--convergence-timeout-s 20`. The next trial starts
only after the previous runner exits and reports cleanup. A remaining runtime
listener, unconfirmed process exit or missing result stops the sequence and retains
its partial summary.

The runner re-executes Python 3.12 in a fresh environment sourced only from
`/opt/ros/jazzy` and this checkout's `packages/omnisim-ros2/install/local_setup.bash`.
It uses CycloneDDS, localhost discovery and domain 83; choose another isolated
domain with `--ros-domain-id` if needed. Ports 6789, 6790 and 8783 must be free.
Existing runtimes and harness sibling files are never reused or terminated.
The stock `nav2_bringup/launch/localization_launch.py` consumes the source map
and OmniSim parameter YAML by absolute paths. Sidecars use bridge port 8783,
bridge-derived simulation clock and explicit LiDAR layer 0. AMCL alone publishes
dynamic `map -> odom`; no scene-TF publisher is started.

By default, each invocation creates `/tmp/husky-amcl-*`. `--output /new/path`
selects a directory that must not already exist. Keep that directory for every
outcome. Default gates require position error **<= 0.10 m** and absolute wrapped
yaw error **<= 0.05 rad** at **both** valid stationary endpoints. These are
conservative functional regression gates, not statistically calibrated accuracy
guarantees. They were selected explicitly for regression use, not fitted to make
an individual trial pass.

## Evidence and verdicts

`result.json` (schema version 2, retaining the endpoint fields) contains:

- `configuration`, `provenance`, `processes`: settings, exact process commands,
  source/runtime hashes, Git revision/status, module paths, physics versions and
  selected environment.
- `world_load`, `backend`, `lifecycle`, `map_metadata`, `scan_geometry`,
  `readiness_timing`, `tf_authority`: essential startup and frame evidence.
- `endpoints.initial` and `endpoints.post_motion`: AMCL pose, covariance and
  simulator timestamp; three independent harness pose acquisitions with host
  monotonic request/response brackets, nearby clock readings and stationary
  velocities; matching scan timestamp, TF at that timestamp and measured errors.
- `movement`: commanded motion, odometry-controlled segment displacements and
  independent net endpoint translation/wrapped rotation.
- `amcl_updates`, `motion_amcl_updates`, `message_counts`: compact scan-update
  evidence. `process_exit`, `cleanup_zero`, `invalid_reasons`, `verdicts` retain
  termination and failure information. Essential process logs sit beside JSON;
  failures also retain `failure.log`. No raw moving-trajectory dataset is saved.
- `trial_id`, `cold_start`: trial identity and fresh-world-load evidence.
- `stationary_observations.initial` and `.post_motion`: valid/invalid sample
  counts, coverage/gaps, maximum observed estimate jumps, `convergence` and
  `jitter` statistics/outcomes. Raw pairs are stored once, in
  `localization_timeseries.jsonl`, including phase, host monotonic time, original
  AMCL simulator timestamp/pose, both ground-truth request brackets/poses,
  matching scan/TF, position error and absolute wrapped yaw error, validity and
  diagnostic reasons. Nonfinite values in invalid evidence are retained as strings
  (`"nan"`, `"inf"`, `"-inf"`) so evidence files remain strict JSON; these strings
  never count as valid numeric samples. No repeated fill values or interpolated
  poses are used.

Verdicts separate performance from evidence integrity:

| Measurement | Localization | Overall | Meaning |
|---|---|---|---|
| PASS | NOT_SCORED | NOT_SCORED | Valid endpoint measurements with `--no-score` |
| PASS | PASS / FAIL | PASS / FAIL | Endpoint accuracy, both convergence checks and both jitter checks scored |
| INVALID | NOT_SCORED | INVALID | Incomplete, stale, inconsistent or failed evidence |

Override either gate with `--max-position-error-m` or `--max-yaw-error-rad`.
Limits apply to both endpoints; yaw uses its absolute wrapped error. Use
`--no-score` to retain measurements without a localization verdict; it disables
both gates even if limit overrides are supplied. The runner does not derive
thresholds from its own trial. Measurement integrity is checked before scoring
and is never relaxed by acceptance limits or unscored mode.
Exit status is 0 for PASS or valid NOT_SCORED, 1 for localization FAIL and 2 for
INVALID. Read `verdicts`, not exit status alone, when assessing performance.

## Convergence and stationary stability

After each existing stopped endpoint snapshot, a lightweight loop requests a
fresh `/request_nomotion_update` approximately every 0.4 host seconds (2.5 Hz).
Without these requests, motion-gated AMCL need not publish at rest. The same
recorder, scan timestamps, TF checks and independent harness queries are reused;
no extra ROS node is introduced. A GT query before and after the update must
bracket its host reception within 1 second, bracket its simulator timestamp,
and show stationary velocity/pose. All GT poses must remain within the existing
stationarity bounds relative to the first observation pose. Clock/scan/odom
freshness checks still apply. Host reception time is labelled as such and never
substituted for the AMCL message timestamp. No moving error pairs are collected.

Convergence is confirmed only after at least six successive valid, strictly
ordered new estimates satisfy both accuracy gates for `--convergence-hold-s`
(default **2.0 host seconds**). Any threshold excursion resets that streak.
`convergence_time_s` measures from observation-loop start to confirmation,
including the hold; `threshold_entry_time_s` identifies the start of the streak.
All duration calculations use host monotonic time, with simulator timestamps
retained only as timing evidence. Inter-sample gaps must be at most **1.0 s**;
the first/last samples must also cover the window boundaries within 1.0 s.
Repeated AMCL timestamps, missing/invalid pairs or larger gaps invalidate the
window rather than proving convergence across missing data.

`--convergence-timeout-s` defaults to **15.0 s per phase**. Collection stops after
confirmed convergence with sufficient jitter coverage, or on timeout. A fully
covered timeout without convergence is FAIL; inadequate evidence is INVALID.
Hold must be at least 2 s and timeout at least hold + 1 s. With the target 2.5 Hz
cadence the passing windows measured on WSL2 took roughly 2–4 s including
acquisition; actual sample intervals were about 0.4–0.7 s. Slow acquisition can instead
produce INVALID. In-flight bounded RPCs can extend wall time past the deadline,
but estimates confirmed after the deadline cannot count as converged.

Jitter reuses the same valid stationary pairs, with at least six samples spanning
at least **2.0 host seconds**. It records mean X/Y, population X/Y standard
deviations, `position_std_m = sqrt(mean((x-mean_x)^2 + (y-mean_y)^2))`, and
maximum radial offset from the mean. Yaw uses `atan2(mean(sin(yaw)), mean(cos(yaw)))`;
`yaw_std_rad` is the RMS wrapped residual from that circular mean, with maximum
absolute wrapped offset also recorded. An undefined circular mean is INVALID.
Defaults are **0.03 m** position RMS and **0.02 rad** yaw RMS, configurable with
`--max-position-jitter-m` and `--max-yaw-jitter-rad`. These conservative functional
gates allow small resampling changes; they are not statistically calibrated noise
bounds. Samples showing robot motion cannot be counted as localization jitter.

The original endpoint snapshots are still scored independently and are not
replaced with later, better estimates. `--no-score` collects the same observations
and computes convergence using the default accuracy gates, but reports acceptance
checks as NOT_SCORED. It still rejects invalid measurement evidence.

`repeat_summary.json` retains each independent verdict, result path, endpoint
errors, convergence and jitter, PASS/FAIL/INVALID/NOT_SCORED counts, PASS rate
(PASS / all attempted runs), and per-phase mean/max errors and convergence times.
Null convergence times are excluded, never converted to zero. Comparability
requires the same commit, source/runtime hashes, basic environment and settings,
unique trial identities/directories and established cleanup. At least three
independent valid trials are required: all PASS yields PASS; any valid FAIL
yields FAIL; missing/corrupt/INVALID results or insufficient runs yield INVALID.
All unscored trials yield NOT_SCORED, never a repeatability PASS. Aggregation reuses the single-trial measurement/scoring functions, recomputes
endpoint errors from acquired poses, and rejects verdicts that contradict the
metrics or thresholds. Each summary record retains the original `trial_verdicts`;
its `overall` reflects aggregation validation. Trial files are never rewritten
by aggregation. Repetition remains manual and is not added to CI.

Historical schema-1 endpoint results have no observation coverage and cannot
pass the full new acceptance checks. They remain unchanged; the endpoint-only
`score()` helper does not certify convergence, jitter or repeatability.

## Stationary measurement contract and limitations

The independent reference is `GET /scene/node/HUSKY`, acquired by the harness
supervisor, rather than `/odom`. The fixture's imported URDF root is
`base_link`; its planar root-to-base offset is zero. World XY is mapped directly
to the explicitly aligned map frame; height is excluded. A tipped/invalid root
orientation invalidates the planar model. The runner checks the source-world
hash, root relationship, measured laser mount and front/left/right ranges.

At each stopped endpoint snapshot it requests `/request_nomotion_update` once, then waits
for a new AMCL estimate stamped by an observed scan inside the stationary
acquisition interval. This handles motion-gated `/amcl_pose` without changing
timestamps or TF tolerance. It does not require endless publications at rest.
Missing fresh estimates, matching scans or scan-time TF invalidate evidence.
Startup also waits for current scan time to enter AMCL's real `map -> odom`
TF history before requesting the initial endpoint; AMCL future-dates that
transform and its earliest entry need not cover the first scan.
Motion must produce at least one additional AMCL update beyond those refreshes.
Recorded `map -> odom` values before/after each refresh expose scan corrections.

Integrity bounds are separate from accuracy thresholds: independent poses must
remain within 2 mm and 0.002 rad over at least one simulator second, with sampled
speeds below 0.005 m/s and 0.005 rad/s. Clock/odom/scan must advance without
rewinds, remain within 0.25 simulator seconds and have receipts newer than one
host second. Three sampled poses do not prove continuous immobility. The harness
endpoint has **no atomic engine acquisition timestamp**; clock readings and host
brackets are evidence of the stationary interval, not synchronized moving poses.
Moving-trajectory accuracy, realistic wheel-odometry drift and global/kidnapped
robot relocalization are outside scope. The measured convergence is stable
localization under a known map/initial pose and ground-truth odometry. The initial
exact seed can produce near-zero error without proving accuracy under uncertainty.
A few independent trials do not constitute a statistical reliability guarantee.

Run the engine-free measurement checks separately:

```bash
python3 -B -m unittest discover -s tests/benchmarks/husky_amcl_localization -p test_measurement.py
```

## Geometry provenance

Source revision: `11514f18e68e4c07473c08c05c338bc07d40615b` (OmniSim v9.2.3).
The YAML/PGM map is an original deterministic rasterization of the collider
dimensions below, released under [Apache-2.0](../../../LICENSE). It contains no
third-party raster imagery or SLAM output; the exact generation procedure is
documented below. New Python sources carry the repository's Apache-2.0 header.
The source world is
[`husky_lidar_walls.omniworld`](../../../projects/samples/demos/worlds/showcase/husky_lidar_walls.omniworld),
SHA-256 `aeb4fd8a4f3852fde2d3ab6781d7deb40ec5eb111513ff66ed0cc54e92df3a2a`.
Its four unrotated static Solids have matching visual and `boundingObject` Boxes:

| DEF | Centre XYZ (m) | Box size XYZ (m) | X footprint (m) | Y footprint (m) |
|---|---|---|---|---|
| FRONT_WALL | (3.05, 0, 0.4) | (0.1, 4.0, 0.8) | [3.00, 3.10] | [-2.00, 2.00] |
| LEFT_WALL | (0, 2.05, 0.4) | (5.0, 0.1, 0.8) | [-2.50, 2.50] | [2.00, 2.10] |
| RIGHT_BOX | (0.12, -1.7, 0.4) | (0.4, 0.4, 0.8) | [-0.08, 0.32] | [-1.90, -1.50] |
| REAR_WALL | (-1.55, 0, 0.4) | (0.1, 2.0, 0.8) | [-1.60, -1.50] | [-1.00, 1.00] |

Each footprint is centre XY plus/minus half the collider size. All four Boxes
span z=0 to 0.8 m. The world's `OmniLinkStage` has `floorSize 10 10` and
`wallHeight 0.05`.
[`OmniLinkStage.proto`](../../../projects/samples/demos/protos/OmniLinkStage.proto)
passes those fields to
[`RectangleArena.proto`](../../../projects/objects/floors/protos/RectangleArena.proto),
which centres the floor at the origin and places its rims outside the floor.

This is a localization map of the four tall objects, not a traversability map.
The floor, 0.05 m rims, robot and visual markers are excluded. The runner checks
that the selected LiDAR layer observes the tall objects rather than the floor
or rims. No SLAM output or identity `map -> world` publication establishes
the alignment used here.

## Grid and coordinate conventions

The map frame is defined to coincide with the authored world's XY origin and
axes: +X, +Y and counterclockwise yaw about +Z. Thus planar map coordinates
equal planar world coordinates. This is an explicit asset convention; it
requires no `map -> world` TF publisher.

The image has 220 columns and 220 rows at 0.05 m/cell, covering
`[-5.5, 5.5) x [-5.5, 5.5)` m. The YAML origin `[-5.5, -5.5, 0.0]` is the
lower-left grid corner in the map frame, with zero grid yaw. It does not move
the map frame's origin. The resolution gives each 0.10 m wall two cells.

For grid indices `i,j` in `[0,219]`, the cell is
`[-5.5 + i*0.05, -5.5 + (i+1)*0.05)` in X and the corresponding interval
in Y using `j`. Its centre is `(-5.5 + (i+0.5)*0.05, -5.5 + (j+0.5)*0.05)`.
For a point inside the grid, `i=floor((x+5.5)/0.05)` and
`j=floor((y+5.5)/0.05)`. PGM column is `i`, but PGM row is `219-j`: the first
image row represents the largest Y. World `(0,0)` belongs to grid cell
`(110,110)`, image column 110/row 109; it is a cell corner, not a cell centre.

- **Occupied (pixel 0):** positive-area intersection with any listed footprint.
  Merely touching a footprint edge does not occupy a cell.
- **Free (pixel 254):** other cells inside `[-5,5) x [-5,5)` m.
- **Unknown (pixel 205):** the surrounding 0.5 m border; no invented outer wall.

With `negate: 0`, occupancy is `(255-pixel)/255`. The YAML's thresholds 0.196
and 0.65 classify 254 as free, 0 as occupied and 205 as unknown
(`50/255 = 0.196078...`, strictly between the thresholds). The image is binary
PGM (`P5`), maximum value 255, with one byte per pixel and no trailing bytes.

Expected occupied grid index ranges are inclusive:

| DEF | Columns i | Grid rows j | PGM rows | Cells |
|---|---|---|---|---:|
| FRONT_WALL | 170..171 | 70..149 | 70..149 | 160 |
| LEFT_WALL | 60..159 | 150..151 | 68..69 | 200 |
| RIGHT_BOX | 108..116 | 72..79 | 140..147 | 72 |
| REAR_WALL | 78..79 | 90..129 | 90..129 | 80 |

The occupied raster expands RIGHT_BOX's unaligned X bounds to `[-0.10,0.35]`
m (0.02 m west, 0.03 m east). Other footprint edges align with the grid.
There is no inflation. The expected totals are 512 occupied, 39,488 free and
8,400 unknown cells; validation must count the generated pixels.

## Exact raster reproduction

From the repository root, this fixed-geometry snippet rewrites only the PGM.
Integer millimetres avoid floating-point ambiguity at shared cell/box edges.
The YAML contains the metadata specified above and uses the image basename,
resolved relative to the YAML directory.

```python
from pathlib import Path

footprints_mm = (
    (3000, 3100, -2000, 2000),   # FRONT_WALL: xmin, xmax, ymin, ymax
    (-2500, 2500, 2000, 2100),   # LEFT_WALL
    (-80, 320, -1900, -1500),    # RIGHT_BOX
    (-1600, -1500, -1000, 1000), # REAR_WALL
)
pixels = bytearray([205]) * (220 * 220)
for j in range(220):
    y0, y1 = -5500 + j * 50, -5500 + (j + 1) * 50
    for i in range(220):
        x0, x1 = -5500 + i * 50, -5500 + (i + 1) * 50
        occupied = any(max(x0, a) < min(x1, b) and
                       max(y0, c) < min(y1, d)
                       for a, b, c, d in footprints_mm)
        inside = -5000 <= x0 and x1 <= 5000 and -5000 <= y0 and y1 <= 5000
        pixels[(219 - j) * 220 + i] = 0 if occupied else 254 if inside else 205
assert {v: pixels.count(v) for v in (0, 254, 205)} == {
    0: 512, 254: 39488, 205: 8400,
}
image = Path("tests/benchmarks/husky_amcl_localization/maps/husky_lidar_walls.pgm")
image.write_bytes(b"P5\n220 220\n255\n" + pixels)
```

## Authored pose

`DEF HUSKY` is authored at `(0,0,0.2)` m, zero yaw, with the mobile bridge on
port 8783. Its nominal planar map seed is `(0,0,0)`, not the YAML grid origin.
This authored pose is not a measurement of the settled robot or its ROS
`base_link`. Independently read `/scene/node/HUSKY`, decode its world orientation
matrix and verify the imported root/base relationship before comparing poses.
Do not use `/odom` as the independent evaluation reference: it is itself the
simulator ground-truth input to AMCL.
