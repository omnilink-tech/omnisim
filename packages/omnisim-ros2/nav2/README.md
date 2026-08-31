# OmniSim × Nav2 (Jazzy) — Husky bring-up

Bring up **Nav2** (ROS 2 **Jazzy**) driving the OmniSim **Husky** — the first Nav2
stack ever run against OmniSim. Everything here sits **on top of** the existing
Tier‑2 ROS 2 sidecar ([`../src/omnisim_ros2`](../src/omnisim_ros2)); it adds **no**
new OmniSim surface and edits **no** existing file.

- **Platform:** Windows 10/11 (engine) + WSL2 **Ubuntu 24.04** (ROS 2 Jazzy)
- **Robot:** Clearpath Husky, diff‑drive, 2D LiDAR (`/scan`) — no cameras
- **Sensor fusion:** none. `/odom` from the bridge is simulator ground truth
  (drift‑free), and the IMU's gyro/accel are dead, so there is no EKF.

> **Why no "cross‑host DDS"?** The Tier‑2 sidecar is the ROS publisher and it talks
> to the Windows engine over **HTTP**. Run **all** ROS 2 nodes inside WSL and DDS
> never leaves WSL — only the sidecar's HTTP calls cross to Windows, through two
> reverse tunnels. The Fast‑DDS profile in [`config/`](config/) is a *fallback only*.

```
Windows-native (prebuilt release)              WSL Ubuntu 24.04 (ROS 2 Jazzy)
┌───────────────────────────┐   HTTP tunnels  ┌────────────────────────────────────┐
│ OmniSim engine + Newton   │◄──────────────► │ omnisim_ros2 Tier-2 sidecar        │
│ harness            :6789  │  (reverse: win  │   pub: /clock /tf /tf_static       │
│ husky mobile bridge :8765 │   dials INTO    │        /odom /scan /imu/data       │
│ world: chat/omnilink_husky│   WSL)          │   sub: /cmd_vel  (plain Twist)     │
└───────────────────────────┘                 │ slam_toolbox → map→odom            │
                                              │ Nav2 (MPPI + SmacPlannerHybrid)    │
                                              └────────────────────────────────────┘
```

---

## Contents

```
nav2/
├── README.md                              ← this runbook
├── params/
│   ├── omnisim_nav2_params.yaml           Nav2 stack (amcl, bt, controller/MPPI,
│   │                                      planner/Smac, costmaps, smoother, behaviors)
│   └── mapper_params_online_async.yaml    slam_toolbox online-async mapping
├── config/
│   └── fastdds_profile.xml                cross-host DDS fallback (normally unused)
└── launch/
    ├── omnisim_slam.launch.py             slam_toolbox + our params, use_sim_time
    └── omnisim_nav2_bringup.launch.py     nav2 navigation (+ optional AMCL) + our params
```

Everything is reconciled to OmniSim's **measured** ROS 2 surface — see
[§ Reconciliation](#reconciliation-guide--omnisim-reality) for the specifics.

---

## 0 · Install (what you need)

### Windows — the engine
Install the prebuilt release (~590 MB) from
<https://github.com/omnilink-tech/omnisim/releases/latest>. This runbook assumes it
is installed at `E:\Proyectos\OmniSim\_engine` (adjust paths if you chose another).

### WSL — Ubuntu 24.04 + ROS 2 Jazzy
```powershell
wsl --install -d Ubuntu-24.04          # Jazzy targets 24.04 (noble), not 26.04
```
Inside the distro:
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-tools \
  ros-jazzy-simulation-interfaces \
  python3-colcon-common-extensions python3-vcstool build-essential git
```
> **WSL DNS gotcha (fresh distros):** if `apt update` fails with
> `Temporary failure resolving 'archive.ubuntu.com'`, the auto‑generated
> `/etc/resolv.conf` is broken. Fix it durably:
> ```bash
> printf '[network]\ngenerateResolvConf = false\n' | sudo tee -a /etc/wsl.conf
> # (from Windows) wsl --shutdown ; then reopen the distro:
> printf 'nameserver 1.1.1.1\nnameserver 8.8.8.8\n' | sudo tee /etc/resolv.conf
> sudo chattr +i /etc/resolv.conf
> ```
> Optional DDS fallback: `sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp`.

### Build the Tier‑2 sidecar (once, inside WSL)
Build on the **native** WSL filesystem (avoid `--symlink-install` on `/mnt/*`):
```bash
cp -r /mnt/e/Proyectos/OmniSim/omnisim/packages/omnisim-ros2 ~/omnisim-ros2
cd ~/omnisim-ros2
source /opt/ros/jazzy/setup.bash
# Tier 2 only — we do NOT need the C++ ros2_control package (Tier 3) for Nav2:
colcon build --packages-select omnisim_ros2
source install/setup.bash
```
The `nav2/` folder (this directory) does **not** need building — its launch files run
by absolute path. Reference them from the checkout, e.g.
`/mnt/e/Proyectos/OmniSim/omnisim/packages/omnisim-ros2/nav2/launch/…`.

---

## 1 · Start the engine + tunnels

### 1a — Engine (Windows terminal, from the install dir)
```powershell
cd E:\Proyectos\OmniSim\_engine
set OMNISIM_URDF_USE_SENSORS=1
:: ^ REQUIRED — without it the imported Husky has ZERO sensors (no /scan).
python -m omnisim harness --port 6789 --supervisor-port 6790
```
Then load the Husky world that runs the **mobile bridge** (serves `/scan` + `/set_velocity`
on `:8765`), light mode for fast stepping:
```powershell
curl -X POST http://127.0.0.1:6789/world/load -H "Content-Type: application/json" ^
  -d "{\"path\":\"projects/samples/demos/worlds/chat/omnilink_husky.omniworld\",\"light\":true}"
```

### 1b — Reverse tunnels (WSL cannot dial the Windows host unprivileged)
Get the WSL IP (from Windows): `wsl -d Ubuntu-24.04 -- hostname -I` → e.g. `172.28.x.y`.

**Inside WSL** (two terminals, or backgrounded):
```bash
T=/mnt/e/Proyectos/OmniSim/omnisim/packages/omnisim-ros2/tools/wsl_harness_link.py
python3 $T wsl --listen-port 6789 --tunnel-port 7799   # harness
python3 $T wsl --listen-port 8765 --tunnel-port 7798   # bridge
```
**On Windows** (two terminals; `--wsl-host` = the IP above):
```powershell
cd E:\Proyectos\OmniSim\omnisim\packages\omnisim-ros2
python tools\wsl_harness_link.py windows --wsl-host <WSL_IP> --tunnel-port 7799 --harness-port 6789
python tools\wsl_harness_link.py windows --wsl-host <WSL_IP> --tunnel-port 7798 --harness-port 8765
```
Now inside WSL, `http://127.0.0.1:6789` is the harness and `http://127.0.0.1:8765`
is the bridge.

> **Throughput ceiling:** the HTTP surfaces are HTTP/1.0 (no keep‑alive) and each
> tunnel adds two Windows sockets per request → ~65 req/s. Keep the sidecar at its
> default rates (~45 req/s). If nodes report the harness unreachable with
> `WinError 10048`, that's ephemeral‑port exhaustion — **lower** rates, don't raise.

---

## 2 · Phase A — Husky baseline  (Milestone M1)

Every ROS terminal below first: `source /opt/ros/jazzy/setup.bash && source ~/omnisim-ros2/install/setup.bash`.

```bash
# Tier-2 sidecar → /clock /tf /tf_static /odom /scan /imu/data ; subscribes /cmd_vel
ros2 launch omnisim_ros2 omnisim_bringup.launch.py \
  harness_url:=http://127.0.0.1:6789 \
  bridge_url:=http://127.0.0.1:8765 \
  use_sim_time:=true

# In another terminal — verify the surface:
ros2 topic hz /scan          # expect ~10 Hz
ros2 topic hz /odom          # expect ~10 Hz
ros2 topic echo /clock --once
ros2 run tf2_tools view_frames   # expect map(after SLAM)→odom→base_link→base_laser

# Teleop (M1): the Husky should move in the sim
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel
```
**Stop here if teleop fails** — debug the tunnels / `OMNISIM_URDF_USE_SENSORS` first.
If `/scan` is missing, the harness was started without `OMNISIM_URDF_USE_SENSORS=1`.

---

## 3 · Phase B — SLAM mapping  (Milestone M2)

```bash
NAV2=/mnt/e/Proyectos/OmniSim/omnisim/packages/omnisim-ros2/nav2
ros2 launch $NAV2/launch/omnisim_slam.launch.py use_sim_time:=true
```
Drive the Husky around with teleop to build the map, then save it:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/omnisim_husky_map \
  --ros-args -p use_sim_time:=true
# → ~/omnisim_husky_map.pgm + ~/omnisim_husky_map.yaml   (M2 achieved)
```
slam_toolbox publishes `map→odom` live, so you can go straight to Phase C **without**
saving/AMCL (SLAM‑first flow).

---

## 4 · Phase C — Nav2  (Milestones M3–M6)

### Flow A — SLAM‑first (recommended, fewest moving parts)
Keep `omnisim_slam.launch.py` running (it owns `map→odom`), then:
```bash
ros2 launch $NAV2/launch/omnisim_nav2_bringup.launch.py \
  use_sim_time:=true localization:=false
```

### Flow B — saved map + AMCL (no SLAM)
```bash
ros2 launch $NAV2/launch/omnisim_nav2_bringup.launch.py \
  use_sim_time:=true localization:=true map:=$HOME/omnisim_husky_map.yaml
```

### Send a goal and verify
```bash
# Costmaps populate from /scan (M4); a global plan appears (M5).
# Send a goal pose (map frame):
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'

# M6 — confirm the robot actually moved, by reading its pose OUT OF THE SIM
# (trust the simulator, not RViz):
curl -s http://127.0.0.1:6789/robots
```
Optional RViz (needs an X server on Windows 10 — e.g. VcXsrv, or WSLg on Win11):
```bash
ros2 launch nav2_bringup rviz_launch.py
```
RViz is convenience only; **no milestone depends on it**.

---

## 5 · Diagnostics (when it breaks)

| Check | Command | Expected for OmniSim |
|---|---|---|
| TF tree | `ros2 run tf2_tools view_frames` | `map→odom→base_link→base_laser`; missing `map→odom` ⇒ SLAM/AMCL not localizing |
| Sim time | `ros2 param get /controller_server use_sim_time` | `true` on **every** node; if `/clock` is silent Nav2 freezes |
| cmd_vel type | `ros2 topic info /cmd_vel` | `geometry_msgs/msg/Twist` ⇒ `enable_stamped_cmd_vel: false` ✓ |
| Frame ids | `ros2 topic echo /scan --once \| grep frame_id` | `base_laser`; `/odom` → `odom` / `base_link` |
| QoS | `ros2 topic info -v /scan` | costmap wants `/scan` BEST_EFFORT, `/tf` RELIABLE |
| Sensors present | `ros2 topic hz /scan` | silent ⇒ harness missing `OMNISIM_URDF_USE_SENSORS=1` |
| Tunnels | (Windows) look for `WinError 10048` | ⇒ ephemeral‑port exhaustion; lower rates |

---

## 6 · Success criteria

| # | Milestone | Evidence |
|---|-----------|----------|
| M1 | Teleop works via ROS 2 | Husky moves with `teleop_twist_keyboard` |
| M2 | SLAM produces a map | `map_saver_cli` writes valid `.pgm` + `.yaml` |
| M3 | Localization | `map→odom` TF published (slam_toolbox or AMCL) |
| M4 | Costmaps populate | obstacles from `/scan` in local/global costmaps |
| M5 | Nav2 plans a path | global plan present after a goal |
| M6 | Nav2 executes | Husky autonomously reaches the goal (verify pose via `/robots`) |

**Any milestone reached is a valid result.** Where it breaks, and why, is the primary
deliverable for OmniSim's beta feedback.

---

## 7 · Verified live bring-up (2026-08-29)

Ran end-to-end on Windows 10 + WSL2 Ubuntu 24.04, OmniSim **v8.1.13** (prebuilt),
ROS 2 Jazzy. **This is the first Nav2-ready ROS 2 surface ever stood up against
OmniSim.**

**✅ Works (verified):**
- Engine + Newton physics (`doctor` VERDICT: READY); Husky world loads; supervisor
  connected; sim clock advances.
- Bridge + sensors live — `list_sensors` returns `base_laser` (Lidar) mounted at
  **`[0.2012, 0, 0.505]`** (only with `OMNISIM_URDF_USE_SENSORS=1`).
- Full ROS 2 surface in WSL: `/scan` (`frame_id: base_laser`, range 0.1–20 m),
  `/odom`, `/clock`, `/imu/data`, `/tf` + `/tf_static`, `/husky/joint_states`.
- TF chain complete and correct: `odom→base_link` tracks motion,
  `base_link→base_laser` = `[0.201, 0, 0.505]`.
**Milestone results (5 of 6 reached):**

| # | Result | Evidence |
|---|--------|----------|
| M1 | ✅ **PASS** | `/cmd_vel linear.x=0.4` drove the Husky **x: 0 → 1.51 m** (pose read from the sim) |
| M2 | ✅ **PASS** | `slam_toolbox` built a map; `map_saver_cli` wrote `omnisim_husky_map.pgm` (50 KB) + `.yaml` (207×242 @ 0.05 m) |
| M3 | ✅ **PASS** | `map→odom` TF published by SLAM (e.g. `[-0.520, 0, 0]`) |
| M4 | ✅ **PASS** | local costmap 60×60 (3 m rolling) + global costmap 207×242 populated from `/scan` |
| M5 | ✅ **PASS** | `ComputePathToPose` (SmacPlannerHybrid) returned a path, `SUCCEEDED`, `error_code: 0` |
| M6 | ✅ **PASS (all-in-WSL Linux)** | `NavigateToPose` drove the Husky **(0,0)→(1.30,0)**, MPPI cmd_vx +0.26, `SUCCEEDED` in ~12 s — see [`REPORT.md`](REPORT.md) §5 |

**Two unlocks were required to get from M1 to M5:**

1. **Run the world at ~1× real time.** By default the harness launches the engine
   `--mode=fast` (~13×), so `/clock` advances in coarse ~8 s sim-time jumps between
   the ~1–2 Hz messages the HTTP/1.0 link sustains, which breaks `slam_toolbox`'s
   scan/TF temporal alignment. Patching the harness to `--mode=realtime` (measured
   **~0.6×**) made `/clock` fine-grained (~95 ms/msg) and SLAM/costmaps/planner all
   worked. *Root cause is OmniSim's HTTP/1.0 throughput; realtime is the practical
   workaround until the harness gets HTTP/1.1 keep-alive or a lidar-only fast reader.*
2. **In Jazzy, `async_slam_toolbox_node` and every Nav2 server are LIFECYCLE nodes.**
   They come up UNCONFIGURED and must be `configure`→`activate`d. `nav2_bringup`'s
   `lifecycle_manager` does this, but under this contended setup its per-node
   transition **times out (~100 ms)** on `collision_monitor` (whose configure is
   slower), so it aborts and respawns in a loop. Driving the nodes to ACTIVE
   **manually** (kill the manager, then `ros2 lifecycle set /<node> configure/activate`)
   brought controller/planner/bt/behaviors/costmaps up stably → M4 + M5.

**Config bugs fixed in this deliverable (both carried from the guide / Jazzy defaults):**
- MPPI `model_dt` must be ≥ controller period. `controller_frequency: 10` (period
  0.1 s) with `model_dt: 0.05` aborts configure — set `model_dt: 0.1`.
- Jazzy's `navigation_launch.py` always starts `collision_monitor`, which aborts
  bringup if it has no config — added a `collision_monitor:` section.

**M6 — reached by running everything inside WSL Ubuntu 24.04 (Linux).** The Windows-engine
+ WSL-ROS split (tunnels, `wslrelay`, firewall, port exhaustion) is what stalled M6; putting
OmniSim *and* ROS 2 in the same Ubuntu, all on `localhost`, closed it. The full recipe +
Linux-specific findings are in [`REPORT.md`](REPORT.md) §5. Key pieces:
build OmniSim (`scripts/install/linux_bootstrap.sh all`), run the harness with
`OMNISIM_NO_WINDOW=1` (else software-Vulkan main-view rendering starves the HTTP server)
+ `OMNISIM_NEWTON_MODEL_DEVICE=cpu` (CPU `mj_step` so the Lidar rays work) + the
`--mode=realtime` patch; use **CycloneDDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) on
every node (FastDDS discovery is unreliable in WSL2); a static identity `map→odom`; the
**lean launch** ([`launch/omnisim_nav2_lean.launch.py`](launch/omnisim_nav2_lean.launch.py))
which starts only the 5 core servers; and this config's global-costmap choices
(rolling window, no `static_layer`, `track_unknown_space:false`) so the planner returns a
valid plan with no map. **Result: the Husky autonomously drove (0,0)→(1.30,0), `SUCCEEDED`.**

**Cross-host gotchas discovered (baked into this runbook):**
- The prebuilt engine's `omnisim-bin.exe` needs `msys64\mingw64\bin\cpp\` on PATH
  (libstdc++/libgcc/libwinpthread live there) — else `LAUNCHER_DLL_NOT_FOUND`.
- The harness (run via system Python) needs `PYTHONPATH` → the source checkout's
  `src\python` for the `omniworld` module.
- **Run the tunnel's WSL listeners on non-default ports** (e.g. 16789/18765). WSL2's
  `wslrelay` mirrors WSL-listening ports onto Windows localhost, so listening on the
  real 6789/8765 in WSL hijacks the Windows harness/bridge.
- Keep sidecar rates low (`clock:=5 state:=1 odom:=4 sensor:=3`, `tf_root_only:=true`)
  or the single-process harness drops its supervisor and the tunnel exhausts Windows
  ephemeral ports (`WinError 10048`).

Exact live launch used:
```bash
ros2 launch omnisim_ros2 omnisim_bringup.launch.py \
  harness_url:=http://127.0.0.1:16789 bridge_url:=http://127.0.0.1:18765 \
  use_sim_time:=true clock_rate_hz:=5.0 state_rate_hz:=1.0 \
  odom_rate_hz:=4.0 sensor_rate_hz:=3.0 tf_root_only:=true
```

---

## Reconciliation (guide → OmniSim reality)

| Guide assumption | OmniSim measured reality | What this config does |
|---|---|---|
| laser frame `laser_frame` | **`base_laser`** (static `base_link→base_laser`) | `base_laser` used as `sensor_frame`; no manual static TF |
| sim publishes topics directly | Tier‑2 **sidecar** publishes over HTTP | run `omnisim_bringup.launch.py` in WSL |
| sensors just exist | **0 sensors** unless `OMNISIM_URDF_USE_SENSORS=1` | set the env var when starting the harness |
| `cmd_vel` maybe stamped | plain `geometry_msgs/Twist` | `enable_stamped_cmd_vel: false` |
| real time | harness ~**13× realtime**, TF ~5 Hz | `use_sim_time: true` + `transform_tolerance` ≈ 1.0 |
| full IMU for EKF | orientation only; gyro/accel dead | no EKF; use `/odom` (ground‑truth) |
| `/scan` normal | single layer auto‑picked from a 4‑layer LiDAR | sidecar `lidar_layer:=-1` (default) |
| AMCL `differential` | Jazzy plugin form | `nav2_amcl::DifferentialMotionModel` |

**Planner note:** the config ships **SmacPlannerHybrid** (per the guide). The Husky is
diff‑drive and can turn in place, so if Hybrid's `minimum_turning_radius` causes
planning failures in tight maps, switch `GridBased` to the commented‑out
**SmacPlanner2D** block in `params/omnisim_nav2_params.yaml`.
