# Omni Quest — research synthesis

How to do outdoor GPS + camera navigation with what OmniSim and OmniLink
already provide, plus the established robotics architecture we're mirroring.
This is the reference that the build (see [`../README.md`](../README.md)) is
designed against.

---

## 1. OmniSim capabilities we reuse

### Sensors (Webots-derived nodes; C++ in `src/omnisim/nodes/`, Python via the `controller` module)

| Sensor | Node / key fields | Controller API | Notes for outdoor nav |
|--------|-------------------|----------------|-----------------------|
| **GPS** | `GPS {}`; world-level `WorldInfo.gpsCoordinateSystem "WGS84"` + `gpsReference <lat> <lon> <alt>` | `enable`, `getValues()` → `[lat,lon,alt]` (WGS84) or `[x,y,z]` (LOCAL), `getSpeed`, `getSpeedVector` | `gpsReference` = lat/lon/alt of the world origin. Examples: `projects/samples/devices/.../gps`, `gps_lat_long`. |
| **Camera** | `Camera { width height fieldOfView recognition segmentation }` | `enable`, `getImage()` (BGRA), recognition + segmentation APIs | The core perception sensor. Object recognition + semantic segmentation available. |
| **InertialUnit** | `InertialUnit {}` | `getRollPitchYaw()` → `[r,p,y]`, `getQuaternion()` | Absolute orientation (a perfect AHRS in sim) → clean heading source. |
| **Compass** | `Compass {}` | `getValues()` → north vector | Magnetometer-style heading; needs declination/offset decoding. |
| **Gyro / Accelerometer** | `Gyro {}` / `Accelerometer {}` | `getValues()` | IMU rates/accels for dead-reckoning + fusion. |
| **Lidar / RangeFinder / Radar** | `Lidar {}` etc. | range image / point cloud / targets | Alternative/added obstacle sensing; RangeFinder = camera-style depth. |

### Robot

**Clearpath Husky** — 4-wheel skid-steer, the recommended outdoor base.
Imported as `URDFRobot { url ".../robots/clearpath/husky_description/urdf/husky.urdf" }`.
Verified kinematics (from `husky_omnilink_bridge.py`):

```
WHEEL_RADIUS_M = 0.1651   HALF_TRACK_M = 0.2854   MAX_WHEEL_SPEED = 6.0 rad/s
motors: front_left_wheel_motor, rear_left_wheel_motor,
        front_right_wheel_motor, rear_right_wheel_motor
skid-steer:  v_left  = (v − ω·HALF_TRACK)/R     v_right = (v + ω·HALF_TRACK)/R
```

Alternatives: Jackal (smaller), Husarion ROSbot, Spot (quadruped, rough terrain).

> **Gotcha (found while building M1):** device nodes (`GPS`/`Compass`/
> `InertialUnit`) added as **children of a `URDFRobot`** mis-bind — Webots
> attached the controller to a robot named after the last device and
> `getDevice` failed. Every camera-equipped Husky world in the repo confirms
> this by using a **separate sidecar `Robot`** (`husky_eye`) that owns the
> Camera and tracks the Husky each tick. So: real device nodes go on a sidecar
> or a native `Robot`, not on the imported URDF body.

### Environment / world conventions

* Outdoor precedent: `samples/demos/worlds/showcase/husky_rocks_traverse.wbt`
  (Husky + `ElevationGrid` hill + `husky_terrain_nav` controller),
  `environments/desert_ruins.wbt`, `forest.wbt`.
* Terrain: `objects/floors/protos/UnevenTerrain.proto` (Perlin), or an
  `ElevationGrid` with `boundingObject USE`.
* Sky/sun (current idiom): `OmniSimSky {}`, `DEF SUN OmniSimSun {}`,
  `DEF SUN_MARKER OmniSimSunMarker {}` + a fill `DirectionalLight`.
* Appearances: `Grass`, `Sand`, `Soil`, `Asphalt`, … under `appearances/protos/`.
* EXTERNPROTO uses the `omnisim://projects/...` URI scheme; `URDFRobot.url` is
  relative to the world file.
* **Project layout:** `worlds/ controllers/<name>/ protos/ Makefile README.md`.
  Python controllers use a no-op `Makefile`; the top-level Makefile recurses
  into `controllers/<name>.Makefile`.

### Headless run contract

`scripts/dev/headless_runner.py <world> --duration N` is canonical, **but** on
this checkout the `webots.exe` launcher misfires (system `WEBOTS_HOME` points
at a different install) and the canonical `omnisim-bin.exe` alias is absent —
so we run `msys64/mingw64/bin/webots-bin.exe` directly with
`OMNISIM_HOME` pointed at this checkout, the bundled `bin` on `PATH`, and a
unique `--port` (the user often has their own OmniSim on 1234). Controller
`print()` doesn't reach `omnisim_log.txt`, so we Tee it to `_last_run.log`.

---

## 2. OmniLink integration (for M4)

OmniLink (a separate repo) is an agent-orchestration + cloud-AI platform. The
`omnilink` Python lib exposes `OmniLinkClient` (cloud LLM chat / memory /
vision), `OmniLinkEngine` (natural-language → command routing), and
`OmniLinkHTTPBridge`.

**Integration pattern.** An OmniSim controller hosts a small HTTP bridge
(`/state`, `/tool`, `/capabilities`, `/prompt`); an external OmniLink agent
drives the robot over JSON — the agent never imports `controller`. Precedents:

* `projects/samples/demos/controllers/husky_omnilink_bridge/` — wheeled base,
  pure-pursuit `goto_cell`, `/state` `/capabilities` `/action` surface (port 6070).
* `agents/production/drone_surveyor/` — flies waypoint circuits, perception per
  waypoint, dedups detections, LLM mission reasoning. **Closest precedent** for
  an Omni-Quest GPS survey agent.
* `agents/bridges/mobile_bridge_stub.py` — swap the bridge endpoint to drive
  real hardware with the same agent code.

**Auth:** `OMNI_KEY` env var (`olink_…`); the relay injects `truststore` for
TLS (handles the AVG MITM cert on this machine). Don't print secrets.

**For M4** we add a `gps_camera_omnilink_bridge` exposing `goto_gps_waypoint`,
`capture_and_geotag`, `read_camera`, and accept NL missions
("survey the field corner A→B, flag anomalies") via the relay + Claude vision.

---

## 3. The reference architecture (ROS 2 Nav2 + robot_localization)

Production outdoor-ground-robot stack; we mirror its shape, staged.

```
                 ┌──────────── STATE ESTIMATION (robot_localization) ───────────┐
 wheel odom ─┐   │ EKF_local (odom→base_link): odom+IMU → smooth, jump-free      │
 IMU ────────┼──▶│ navsat_transform_node: NavSatFix(WGS84) → map XY              │──▶ map→odom→base_link
 GPS ────────┘   │ EKF_global (map→odom): + GPS → absorbs GPS jumps              │
                 └──────────────────────────────────────────────────────────────┘
 GPS waypoints (lat/lon) ──fromLL──▶ map goals ──▶ global planner ──▶ path
 camera ──▶ perception (seg/depth) ──▶ obstacle/voxel layer ──▶ local costmap
                                                              ▼
                                       local controller (pure pursuit / DWB) ──▶ cmd_vel
```

The single most important structural choice is the **two-EKF + navsat_transform**
topology: a *local* EKF (odom+IMU → continuous `odom→base_link`, what the
controller integrates against) and a *global* EKF (+GPS → `map→odom`, which
absorbs GPS jumps without ever discontinuing `base_link`). GPS waypoints are
followed by converting each lat/lon to a map-frame goal (`fromLL`) before the
normal planner/controller.

### Geodesy — lat/lon ⇄ local metric frame

Full path (what `navsat_transform_node` does): geodetic → ECEF → ENU about a
datum. WGS-84: `a = 6378137 m`, `e² = 6.69437999e-3`.

```
N = a / √(1 − e²·sin²φ)
X = (N+h)·cosφ·cosλ   Y = (N+h)·cosφ·sinλ   Z = (N(1−e²)+h)·sinφ
ENU = R(φ₀,λ₀) · (ECEF − ECEF₀)
```

First-cut (what `geo.py` uses) — equirectangular with **local radii** (cm-accurate over a campus):

```
M = a(1−e²)/(1−e²sin²φ₀)^{3/2}   (meridian)     N = a/√(1−e²sin²φ₀)   (prime vertical)
E = N·cosφ₀·(λ−λ₀)               North = M·(φ−φ₀)      (φ,λ in radians)
```

**Frame:** ROS/OmniSim are **ENU**, yaw 0 = East. Many IMUs/datasheets are NED,
yaw 0 = North — a π/2 `yaw_offset` + magnetic declination must both be set or
the robot drives off at a constant angle. **Heading is the hard part:** GPS
gives position but not orientation. Options: magnetometer + declination (noisy
near motors); dual-GPS moving-baseline (best outdoors); or drive-straight-to-
infer-heading at startup.

### Control laws

**Heading-P** (M1 — simplest viable):
```
psi_d = atan2(N_goal−N, E_goal−E)   e = wrap_pi(psi_d − psi)
omega = K_p·e        v = v_max·max(0, cos e)      reached: dist < tol (0.3–1.0 m)
```

**Pure pursuit** (M2 — geometric; diff-drive form):
```
curvature κ = 2·sin(α)/L_d        steering δ = atan(2·L·sin α / L_d)
diff-drive: omega = v·κ = 2·v·sin(α)/L_d      lookahead L_d = k_v·v + d₀ (clamped)
```
Too-small `L_d` → oscillation; too-large → corner-cutting.

**Stanley** (front-axle alternative): `δ = ψ + atan(k·e/(v+k_s))` — nulls
cross-track error harder, twitchier at low speed.

### Sensor fusion

Wheel odom = smooth but drifts; IMU = high-rate but biased; GPS = absolute but
noisy (1–20 m consumer, decimetre RTK) and slow (2–10 Hz). An **EKF** predicts
with the motion model and corrects per-measurement by covariance: GPS anchors
absolute position (kills drift), odom+IMU fill in smooth high-rate pose between
fixes and ride through dropouts. Simpler M2 first cut — dead-reckon between
fixes + complementary blend:
```
x += v·cosψ·dt ; y += v·sinψ·dt ; ψ += ω·dt
on GPS fix:  x ← (1−α)·x_dr + α·x_gps   (α≈0.1–0.3)
yaw:         ψ = β·(ψ+gyro·dt) + (1−β)·ψ_mag   (β≈0.98)
```

### Camera outdoors — what's realistic to simulate

In the GPS+IMU+odom stack the camera contributes **nothing to global pose** —
it feeds perception → obstacle/free-space → costmap. Order of sim difficulty:

1. **Depth → obstacle layer** (do first) — stereo/RGB-D → point cloud → voxel
   costmap. Sim gives ground-truth depth nearly free. **Model depth noise** —
   obstacle-detection precision collapses under realistic noise even at high
   recall, so clean-depth results don't transfer.
2. **Semantic segmentation** for traversability/free-space (drivable ground vs
   obstacle); even ground-truth sim masks are a useful stub before a real net.
3. **Lane/row/path following & visual servoing** — steer to a detected path centre.
4. Monocular depth — research add-on, not a baseline.

### Pitfalls to design in deliberately

* GPS noise/multipath — never feed raw GPS to control; always through the
  fusion/`map→odom` layer so `base_link` stays continuous. Simulate noise + dropouts.
* **Heading initialization** is the #1 real-world failure — position converges
  from one fix, heading does not. Gate autonomy on a heading-init maneuver.
* ENU vs NED + datum + declination — a constant-angle drift bug if mis-set.
* Waypoint-reached tolerance — too tight → circles a noisy goal; too loose →
  sloppy. 0.3–1.0 m outdoors, scaled to GPS accuracy.
* Pure-pursuit lookahead — speed-scale and clamp it.

### Key sources

Nav2 *Navigating using GPS Localization* (dual-EKF, navsat_transform,
FollowGPSWaypoints/fromLL); `robot_localization` `navsat_transform_node`
(ENU/REP-103, datum, `magnetic_declination_radians`, `yaw_offset`);
automaticaddison ROS 2 GPS+robot_localization; BlackCoffee Robotics GPS
localization; pure-pursuit derivation (κ=2sinα/L_d); Stanley/pure-pursuit/MPC
comparison; geodetic→ECEF→ENU (govert gist); RGB-D obstacle detection for ag
robots (PMC8399919, the depth-noise cliff); traversability-from-RGB
(arXiv 2009.07565).

---

## 4. Decisions for Omni Quest

| Decision | Choice | Why |
|----------|--------|-----|
| Robot | Clearpath Husky | Recommended outdoor base; kinematics + pure-pursuit precedent in-repo. |
| GPS coords | WGS-84 lat/lon | Realistic; the project is *about* GPS. |
| GPS source (M1) | **Modelled** on ground truth + noise | Real device nodes mis-bind on the URDFRobot; modelling also gives full noise control for M2 fusion. Auto-upgrades to real nodes if present. |
| Heading (M1) | IMU-style yaw | Clean; sidesteps the heading-init problem until M5. |
| Camera (M3) | **Real** rendered `Camera` on a `husky_eye` sidecar | Honors "cameras" literally; the proven mounting pattern. |
| Geodesy | Equirectangular w/ local radii | cm-accurate over a campus, round-trip exact, simple. |
| Control | heading-P (M1) → pure pursuit (M2) | Nav2 build-order: reach waypoints first, make it credible next. |
| Backend | ODE/default | Stable for wheeled outdoor; Newton has known caveats. |
| Agent layer (M4) | OmniLink bridge + `drone_surveyor`-style mission agent | Established integration; NL missions + Claude vision. |
