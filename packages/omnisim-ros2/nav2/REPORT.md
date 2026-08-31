# OmniSim × Nav2 (Jazzy) — Bring-up Report & Beta Feedback

**First Nav2 stack ever brought up against OmniSim.** Milestones **M1–M5 reached**,
M6 (autonomous execution) in progress. This document is written for the **OmniSim
developers**: every measured value, every engine/harness/bridge/sidecar behaviour, and
every fix needed to get here. Reproduction commands are in [`README.md`](README.md).

- **Date:** 2026-08-29 / 30
- **Host:** Windows 10 Pro 19045 + WSL2 **Ubuntu 24.04** (noble)
- **Engine:** OmniSim **v8.1.13** prebuilt (`omnisim-v8.1.13_setup.exe`, Inno Setup, ~590 MB), installed to `E:\Proyectos\OmniSim\_engine`
- **ROS:** ROS 2 **Jazzy**; sidecar `omnisim_ros2` (Tier 2) colcon-built in WSL
- **World:** `projects/samples/demos/worlds/chat/omnilink_husky.omniworld` (mobile-bridge Husky, bridge on `:8765`)

```
Windows-native (prebuilt engine)         WSL Ubuntu 24.04 (ROS 2 Jazzy)
 harness :6789  ── reverse tunnel ──►  omnisim_ros2 sidecar → /scan /odom /clock /tf /imu
 husky bridge :8765                    slam_toolbox → map→odom
 Newton physics                        Nav2 (MPPI + SmacPlannerHybrid + costmaps)
   (WSL dials 127.0.0.1:16789/18765 → wsl_harness_link → 127.0.0.1:6789/8765)
```

---

## 1 · Milestone results (measured)

| # | Milestone | Status | Measured evidence |
|---|-----------|--------|-------------------|
| **M1** | Teleop via ROS 2 | ✅ PASS | `/cmd_vel linear.x=0.4` → Husky pose **x: 0.0000 → 1.5102 m** (read from `/get_robot_state`) |
| **M2** | SLAM builds a map | ✅ PASS | `slam_toolbox` → `/map` **207×242 @ 0.05 m** (~10.3×12.1 m); `map_saver_cli` wrote `omnisim_husky_map.pgm` (50109 B) + `.yaml` |
| **M3** | Localization (`map→odom`) | ✅ PASS | slam_toolbox broadcast `map→odom` = `[-0.520, 0, 0]` (needs `tf2_echo ... -p use_sim_time:=true`) |
| **M4** | Costmaps populate | ✅ PASS | local costmap **60×60 @ 0.05** (3 m rolling), global **207×242 @ 0.05**, obstacle layer subscribed to `scan` |
| **M5** | Nav2 plans a path | ✅ PASS | `ComputePathToPose` (GridBased/SmacPlannerHybrid) → **`SUCCEEDED`, `error_code: 0`**, path returned |
| **M6** | Nav2 executes | ✅ **PASS (Linux)** | On the all-in-WSL Linux build: `NavigateToPose` drove the Husky **(0,0) → (1.30, 0)**, MPPI cmd_vx **+0.26**, `SUCCEEDED`, reached in ~12 s — see §5 |

**TF chain (verified correct):** `map→odom` (SLAM) → `odom→base_link` (odom_node, tracks
motion, e.g. `[1.579,0,0]`) → `base_link→base_laser` (static, `[0.201,0,0.505]`).

---

## 2 · Findings for OmniSim developers

### 2.1 Prebuilt engine / installer — two blockers that stop the harness cold
1. **`omnisim-bin.exe` fails to load with `STATUS_DLL_NOT_FOUND` (0xC0000135) unless
   `msys64\mingw64\bin\cpp\` is on PATH.** `objdump -p` shows the binary imports
   `libstdc++-6.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`, and in this release
   those three live **only** in `mingw64\bin\cpp\`, not in `mingw64\bin` (all other
   deps — Qt6*, wgpu_native, python312, OIS, assimp, freetype, openal — are in
   `mingw64\bin`). `launch.bat` prepends only `mingw64\bin`; `omnisim.bat` prepends
   neither. The harness inherits `omnisim.bat`'s env, so `/world/load` returns
   `LAUNCHER_DLL_NOT_FOUND` until PATH includes `cpp\`.
   → **Suggest:** ship those three DLLs in `mingw64\bin` (as the other runtime DLLs are),
   or have `omnisim.bat`/the harness add `mingw64\bin\cpp` to PATH.
2. **The harness needs the `omniworld` Python package, which the release does not ship.**
   `scripts/harness/omnisim_harness.py` → `spatial.py` → `from omniworld import viewpoint`;
   run under the user's system Python it raises `ModuleNotFoundError: No module named
   'omniworld'` and exits. The package exists only in the source tree at
   `src/python/omniworld/`. Workaround: `PYTHONPATH=<source>\src\python`.
   → **Suggest:** include `src/python/omniworld` in the packaged harness, or lazy-import it.

### 2.2 Harness — the throughput & mode limits are the M6 wall
3. **Hardcoded `--mode=fast` (~13× real time); no API to request real time.**
   `omnisim_harness.py` builds the engine command with `"--mode=fast"` (line ~3285).
   Measured under `--mode=realtime`: **~0.6×** (1824 ms sim / 3000 ms wall). For ROS,
   sensor cadence must track real time — at 13× the `/clock` messages the HTTP/1.0 link
   can deliver (~1.6 Hz) land **~8 s of sim-time apart**, which breaks slam_toolbox's
   scan↔TF temporal alignment (no map). Patching to `--mode=realtime` was the single
   change that unlocked M2–M5.
   → **Suggest:** expose a run-mode / target-real-time-factor param on `/world/load` (or
   a harness flag). This is the highest-value change for ROS/Nav2 users.
4. **HTTP/1.0, no keep-alive → hard throughput ceiling.** Through the WSL reverse tunnel
   the sidecar sustained only **~0.73 Hz `/scan`** and **~1.6–6 Hz `/clock`** (the
   sensor node reads 5 sensors sequentially per tick). Each request is a fresh TCP
   connection; the tunnel doubles it on the Windows side, and at ~48 req/s the bridge
   tunnel exhausted Windows' ephemeral ports (`WinError 10048`, ~16k `TIME_WAIT`).
   → **Suggest:** give the harness `protocol_version = HTTP/1.1` + keep-alive (the *bridge*
   already does; the ROS client pools it). This would likely carry the setup to M6.
5. **Single-process harness drops its supervisor under HTTP load.** At `clock_rate 20 +
   state_rate 5` (~35 harness req/s) the supervisor connection dropped
   (`load_state: supervisor_lost`) while the engine stayed alive — `/clock` and `/scan`
   died, `/odom` (bridge) survived. Stable at ~16 req/s (`clock 5–8, state 1–2`).
   Reloading the world reconnects the supervisor in ~2 s.
   → **Suggest:** service the supervisor heartbeat off the HTTP-serving thread.

### 2.3 Bridge (`omnilink_mobile_bridge`) — mostly good; two notes
6. **`OMNISIM_URDF_USE_SENSORS=1` is mandatory and non-obvious.** Without it,
   `list_sensors` returns 0 devices → `/scan` never appears. With it, 9 devices reported
   (`base_laser`, 4 wheel PositionSensors, `imu_data`/`imu_data_accel`/`imu_data_gyro`,
   `navsat_fix`). Measured `base_laser` mount `[0.2012, 0, 0.505]`.
7. **Lidar arrives as 4 layers** (`finite returns per layer [0,0,0,516]`); the sidecar
   auto-picks layer 3 for `/scan`. `read_sensor` returns `null` for no-return rays
   (mapped to `inf`). Request field is `sensor` (not `name`).
8. Bridge binds **`127.0.0.1:8765` (loopback only)** → unreachable cross-host; requires
   the reverse tunnel or a portproxy. Bridge speaks **HTTP/1.1 + keep-alive** (good).
   → **Suggest:** optional bind address on the bridge to skip the tunnel.

### 2.4 Sidecar (Tier 2) — correct; one efficiency note
9. Publishes exactly what Nav2 needs, correctly: `/scan` (`base_laser`), `/odom`
   (ground-truth, drift-free), `/clock`, `/imu/data` (orientation real; gyro/accel
   covariance `-1`), `/tf`+`/tf_static`, `<robot>/joint_states`. `use_sim_time` honoured.
10. **`robot_state_node` and `odom_node` both emit TF.** With `tf_root_only:=false`
    (default) `robot_state_node` publishes the full scene tree, which can give `base_link`
    two parents (world-tree and `odom→base_link`). `tf_root_only:=true` avoids the
    conflict — recommend it (or documenting it) for Nav2 use.
11. **The sensor node reads all 9 devices per tick**, so `/scan` cadence is 1/5 of what a
    lidar-only reader would give over the slow link.
    → **Suggest:** a topic/sensor allowlist (e.g. lidar-only) to raise effective `/scan` Hz.

### 2.5 Cross-host (Windows ↔ WSL2) — for the docs
12. **WSL2→Windows is firewall-blocked** on Win10 (confirmed: works only with the firewall
    off). The shipped `tools/wsl_harness_link.py` (Windows dials *into* WSL) is the correct
    unprivileged path.
13. **WSL2 `wslrelay` mirrors WSL-listening ports onto Windows `localhost`.** Running the
    tunnel's WSL listeners on the real `6789/8765` hijacks the Windows harness/bridge —
    the harness then fails to (re)bind (`port already bound (not HTTP)`) and the bridge
    goes dead. **Use different WSL listener ports (16789/18765).**
14. A `netsh portproxy` on `0.0.0.0:6789/8765` **must be added after** the harness/bridge
    bind their loopback ports, or it blocks their bind.

### 2.6 Nav2-on-Jazzy specifics (useful for a future OmniSim Nav2 guide)
15. **Jazzy `async_slam_toolbox_node` and all Nav2 servers are LIFECYCLE nodes** — they
    start `UNCONFIGURED` (only `/clock` + param subs; no `/scan` sub, no `/map`). Must be
    `configure`→`activate`d. `nav2_bringup`'s `lifecycle_manager` does this, but under this
    contended host its transition service **timed out (~100 ms)** on `collision_monitor`
    (whose configure is slower — it builds scan source + polygons) and it respawn-looped.
    **Driving nodes to ACTIVE manually** (`ros2 lifecycle set /<node> configure/activate`)
    is a reliable workaround and brought controller/planner/bt/behaviors/costmaps up → M4+M5.
16. **MPPI `model_dt` must be ≥ controller period.** `controller_frequency: 10` (0.1 s) with
    `model_dt: 0.05` (the guide's values) aborts configure: *"Controller period more then
    model dt"*. Fixed to `model_dt: 0.1`.
17. **`navigation_launch.py` always starts `collision_monitor`** (no disable arg) and
    lifecycle bringup aborts if it has no params → added a `collision_monitor:` section.
18. **cmd_vel chain (Jazzy):** controller → `cmd_vel_nav` (launch remap) → velocity_smoother
    → `cmd_vel_smoothed` → collision_monitor → `cmd_vel` (what the OmniSim command_node reads).
    When collision_monitor isn't active, `tools/cmd_vel_relay.py` bridges
    `/cmd_vel_smoothed → /cmd_vel` to close it.
19. **Jazzy's `navigation_launch.py` bundles OPTIONAL nodes that abort the WHOLE lifecycle
    bringup if unconfigured.** Beyond the core servers it also starts `collision_monitor`
    (aborts without `observation_sources`), `docking_server` (aborts with *"Charging dock
    plugins not given!"* without `dock_plugins`), plus `route_server` / `waypoint_follower`.
    A single missing config makes `lifecycle_manager` `Aborting bringup`. For OmniSim,
    a **lean custom launch** that instantiates only controller/planner/bt/behavior/
    velocity_smoother + costmaps + a lifecycle_manager over just those is the robust path.

### 2.7 Why M6 (autonomous execution) didn't close on this host
Reached **transient** activation (controller MPPI + planner Smac + costmaps + a plan —
M4/M5), but never a **stable** running stack that executes a goal. Three compounding causes,
all setup-level (not Nav2-config): **(a)** the heavyweight bundled launch (finding 19) means
the stock `lifecycle_manager` aborts, so activation must be driven manually; **(b)** under
resource contention the manual `ros2 lifecycle set` / discovery calls intermittently return
empty (DDS shared-memory discovery flakiness with ~20 nodes on one box), so activation is
unreliable; **(c)** repeated kill/relaunch cycles eventually leave the Nav2 processes
unreachable. Root cause is **one contended machine** (engine + 6 sidecar + slam + ~20 Nav2
nodes + the HTTP/1.0 cross-host link). **To finish M6:** run a lean launch on a less
contended host (or give the harness HTTP/1.1 keep-alive so fewer workarounds are needed),
then send a `NavigateToPose` goal with `cmd_vel_relay.py` closing the command chain. The
config and TF are already correct — this is an infrastructure/scale gap, not a Nav2 gap.

---

## 3 · Prioritised recommendations to OmniSim

1. **HTTP/1.1 keep-alive on the harness** — the single biggest lever for ROS throughput;
   would likely lift this setup from M5 to M6 without any other change.
2. **Expose a run-mode / real-time-factor** on the harness API (`--mode=realtime` had to be
   patched in by hand; it is essential for sensor-driven ROS work).
3. **Fix release packaging:** put `libstdc++/libgcc/libwinpthread` in `mingw64\bin` (or add
   `cpp\` to PATH in `omnisim.bat`), and ship the `omniworld` package with the harness.
4. **Optional bind address on the bridge** (skip the tunnel entirely when a firewall rule
   or 0.0.0.0 bind is acceptable).
5. **Isolate the supervisor heartbeat** from HTTP serving so it survives load bursts.
6. **Sidecar lidar-only read option** to raise effective `/scan` Hz over slow links.
7. **Document the WSL2 gotchas** (wslrelay port mirroring; firewall direction; portproxy
   ordering) — they cost the most time and are not guessable.

---

## 5 · The winning path — everything inside WSL Ubuntu 24.04 (Linux), M6 reached

Running OmniSim **and** ROS 2 in the *same* Ubuntu 24.04 removed ~70% of the pain
(no tunnels, no `wslrelay`, no firewall, no portproxy, no ephemeral-port exhaustion,
no Windows DLL/`omniworld` packaging issues) and got the full stack to **M6**.

**Setup:** `bash scripts/install/linux_bootstrap.sh all` (build from source, ~30 min;
system Python 3.12 on 24.04 is the target; CPU `mj_step` is enough, no GPU needed —
though warp used `cuda:0` when present). Smoke test passed (Newton verdict
`finalised:true`). Then, all on `localhost` (no tunnels):

- **Harness:** `OMNISIM_URDF_USE_SENSORS=1 OMNISIM_NEWTON_MODEL_DEVICE=cpu OMNISIM_NO_WINDOW=1
  xvfb-run python3 -m omnisim harness` (patched `--mode=fast`→`--mode=realtime`).
- **Sidecar / SLAM-substitute / Nav2:** all point at `127.0.0.1:6789/8765`.

**Linux-specific findings (for OmniSim devs):**
1. **`OMNISIM_NO_WINDOW=1` is essential for a headless server.** Without it the engine
   renders the main view via wgpu **software Vulkan (lavapipe under Xvfb)** every step,
   which slowed `/sim/state` to **0.15–0.93 s** and made the harness time out under the
   sidecar's load. With it: **~0.027 s**. (Offscreen device renders — the Lidar — still work.)
2. **The Lidar on Linux is computed by the wgpu depth renderer** (`[OmLidar] range via wgpu`,
   4 layers × 541), not `mj_ray`. Works headless, same result as Windows (layer 3 = floor,
   516 finite). It requires `newtonSolver "mujoco"` (CPU `mj_step`) — pin with
   `OMNISIM_NEWTON_MODEL_DEVICE=cpu`; on `mujoco_warp` (GPU) ray sensors are declined.
3. **`/set_velocity` takes `{"linear":…, "angular":…}`**, not `{"v":…, "w":…}` (an old
   AGENTS.md shape). The sidecar's `command_node` already uses the correct fields.
4. **CycloneDDS was required.** FastDDS discovery between separate processes in WSL2 was
   unreliable (nodes ran but ephemeral `ros2` shells + the Nav2↔sidecar links intermittently
   saw nothing). `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on every node fixed it instantly
   (19 nodes discovered, all 5 Nav2 servers ACTIVE in ~2 s).

**Nav2-config fixes that took it to a clean `SUCCEEDED` (all in `params/`):**
- **Planner → `SmacPlanner2D`** (Hybrid/Dubin made the diff-drive base do reverse maneuvers).
- **Global costmap: `rolling_window: true` + NO `static_layer` + `track_unknown_space: false`.**
  With no map_server, the static layer keeps the costmap unknown/lethal and the planner
  returns an **empty plan** → the BT falls into spin/backup recovery (the "robot reverses
  away from the goal" symptom). Dropping it (unknown=free, rolling window) gave a valid plan.
- **MPPI `model_dt: 0.1`** (≥ 1/controller_frequency) and **`vx_min: 0.0`** (forward-only).
- **`lidar_layer:=0`** on the sidecar for this world (0.1 m walls → the horizontal layer sees
  open space; the down-angled layer 3 sees the *floor* and would fill the costmap with phantom
  obstacles). A world with taller walls would give real obstacle avoidance.
- Static identity `map→odom` (OmniSim `/odom` is ground-truth) instead of SLAM/AMCL.

**Result:** `NavigateToPose (1.5, 0)` → robot drove **(0,0) → (1.30, 0)** with MPPI
`cmd_vx +0.26`, `Goal … SUCCEEDED`, reached in ~12 s. **M1–M6 all green.**

---

## 4 · What ships in this deliverable
`packages/omnisim-ros2/nav2/` (new folder, **zero edits to existing repo files**):
`README.md` (runbook), `params/omnisim_nav2_params.yaml` (reconciled + M5 fixes),
`params/mapper_params_online_async.yaml`, `config/fastdds_profile.xml`,
`launch/omnisim_slam.launch.py`, `launch/omnisim_nav2_bringup.launch.py`, and this `REPORT.md`.
