# Webots-lane bring-up record (Phase W execution path)

**2026-07-31.** What is installed on the WSL2 side, with hashes; how the
execution path was proven end-to-end; and the exact numbers the bring-up run
produced. Companion to [`docs/developer/webots-control-baseline.md`](../../../../../docs/developer/webots-control-baseline.md)
(the recipe) and [`docs/developer/agent-edge-validation-plan.md`](../../../../../docs/developer/agent-edge-validation-plan.md)
Phase R item 4 (the mandate). The numbers below are **bring-up telemetry, not
benchmark results** — one 2-robot validation world, no conditions, no n.

## 1. The install (WSL2 `Ubuntu-22.04`, on this machine)

| item | value |
|---|---|
| distro | WSL2 Ubuntu 22.04.5 LTS (jammy), 927 GB free at install time |
| install root | `/opt/upstream-webots/R2025a` (dir `webots/` in the archive, renamed at unpack) |
| archive | `webots-R2025a-x86-64.tar.bz2` from `github.com/cyberbotics/webots/releases/download/R2025a/` |
| archive sha256 | `c5127fb4206c57a5ae5523f1b7f3da8b670bc8926d9ae08595e139f226f38c38` — **matches the baseline doc's pinned value** (upstream publishes no checksums; this is our computed pin) |
| provenance of the installed tree | the tree was first unpacked 2026-07-26 (baseline-doc session) and the archive deleted; re-verified 2026-07-31 by re-downloading, re-hashing (match), and `tar --compare` against `/opt/upstream-webots/R2025a` — **zero content differences** (the only diff line was the compare symlink itself) |
| version | `R2025a` (`resources/version.txt`; also echoed into every run's `process.json`) |
| runtime deps | upstream's own `scripts/install/linux_runtime_dependencies.sh` — **not shipped in the binary tarball**; fetched from the `R2025a` tag (sha256 `ecd0be4b0aa20cde49194516c53e1548979b98f9f8723dae01f4ebd219507105`), ran to completion (all packages already present), plus `xvfb` 2:21.1.4-2ubuntu1.7~22.04.16 |
| asset pre-seed (baseline §5 trap 1) | `assets-R2025a.zip` (871,583,665 bytes, **4,893 flat sha1-named files**), sha256 `852e776380f333b593ecb68f33c189e7efb2f35a0014027a1cdc60f8339a3f8f` (no upstream checksum exists; recorded here), extracted no-overwrite into `/root/.cache/Cyberbotics/Webots/assets/` → cache now 842 MB / 4,893 entries. `EXTERNPROTO` loads resolve from this cache, so worlds run without network |
| `WEBOTS_HOME` hygiene (baseline §2) | **per-process only** — prefixed onto the one launch command line. Verified absent from `/root/.bashrc`, `/root/.profile`, `/etc/environment`, `/etc/profile.d/*`, `/etc/bash.bashrc` after install. The Windows installer was **never** run (baseline §1); the Windows-side `WEBOTS_HOME`/`OMNISIM_HOME` were never touched |

WSL note that shaped the launcher: the distro auto-terminates when its last
client exits, and **`/tmp` is cleaned on the next start** — so the launcher
does its whole setup→run→copy-back inside **one** `bash` script invocation and
treats the WSL workdir as disposable.

## 2. The pieces (all new, all under this directory)

| file | role |
|---|---|
| `launcher.py` | Windows-side launcher: resolves the world's project, writes the injected sibling world (`_agentbench_<stem>.wbt`, same marker prefix as the OmniSim arm), assembles a WSL-side project copy, runs the baseline §3 invocation under `timeout` + `xvfb-run`, copies the artifact dir back, writes `process.json`. Ports are confined to **1500–1510** (OmniSim auto-scans 1234–1244; baseline §6). Pure functions for everything WSL-independent |
| `webots_lane/controllers/agentbench_webots_recorder/` | the Webots-side Supervisor recorder (upstream `controller` API only). t=0 frozen roster scan (`synchronization TRUE`), per-basic-timestep pose sampling, `getContactPoints()` world-point pairing across robot subtrees for the first N steps, and **termination**: upstream has no `--duration`/auto-exit, so the recorder writes `completion.json` and calls `simulationQuit(0)` — which is also what makes `--log-performance` flush (`worldClosed()`) |
| `webots_lane/controllers/bringup_drive/` | sensor-free per-robot arc drive for the bring-up world (trap 2: no lidar) |
| `webots_lane/worlds/bringup_pioneer.wbt` | 2× Pioneer 3-AT (baseline §4: upstream ships no Husky; Pioneer 3-AT is the 4-wheel skid-steer analogue) from upstream assets only; yaws 2.9 / −0.4 — **never ±π** (trap 3) |
| `test_launcher.py` | 24 WSL-free unit tests (path translation, port discipline, invocation shape, injection, run-script ordering, artifact-dir contract vs `recording.read_run`) |

Artifact set produced per run (exactly what `recording.read_run` parses):
`trajectory.json`, `roster.json`, `contacts.json`, `completion.json`,
`process.json`, `stdout.log`/`stderr.log`, `perf.txt`, plus `version.txt` and
the injected world copy.

## 3. End-to-end proof (bring-up telemetry — NOT benchmark results)

`launcher.launch(bringup_pioneer.wbt, run_dir, duration=10.0)`, two runs, both
first-attempt clean:

- exit code **0** (clean `simulationQuit`, not a kill), `timed_out=false`,
  wall **5.89 s / 6.37 s** for the webots process (load + 10 s sim + quit)
- **626 samples** (625 steps @ 16 ms), `recorded_s = 10.0` simulated seconds
- both robots moved and stayed finite: net displacement **3.80 m / 3.86 m**
  (path length 4.17 m / 4.18 m) in 10 s
- roster: 3 bodies — `rectangle arena` (Solid, no physics), 2× `Pioneer3at`
  (Robot, **n_joints = 4**, controller `bringup_drive`, has_physics)
- contacts: `supported=true`, 10-step window, `total_observed=24` (wheel–floor),
  `distinct_named=0` (no robot–robot contact — correct)
- perf log non-empty (1,052 bytes) — the independent finalize witness
- console: controller starts counted — `{bringup_drive: 2, agentbench_webots_recorder: 1}`;
  zero `ERROR:` lines

Adapter confirmation against these REAL artifacts:

- `recording.read_run(run_dir)`: **all five JSON artifacts + console + perf
  found; `missing == {}`, `errors == {}`**; `to_arrays` clean (no problems)
- `evidence.build_bundle(A1, robot_identity="husky", robot_proto="Pioneer3at")`:
  builds; frozen t=0 inventory, trajectory aligned to roster, declared-analogue
  identity works (`declared_count=2`, `robot_class=[False, True, True]`),
  attribution `ode`/`R2025a`, `reached_finalize=True`, `live_load_ok=True`
- `adapters.check_bundle`: **one** finding — no world-space AABBs (upstream has
  no bounds API and this recorder does not compute them; a documented gap the
  bundle publishes, see §4)
- `a1_core.grade`: runs end to end. Outcome FAIL, correctly — a 2-robot
  bring-up world graded against the 10-robot A1 task (A1.9 clean-exit and
  A1.10 attribution pass). That is the grader working, not a score.

Tests: `pytest tests/benchmarks/agentbench/adapters/webots -q` →
**72 passed** (48 existing adapter tests + 24 new launcher tests), no WSL
needed.

## 4. Known gaps / caveats (state, don't bury)

1. **No world-space AABBs *from the engine*.** Upstream's Supervisor API has no
   bounds query, so every AABB on this arm is one a controller **computed**
   from the body's own geometry and world pose. Two channels do it, and they
   are deliberately kept apart:

   * `agentbench_aabb_prober` — a **separate measurement launch** of the same
     world text (`task_support.probe_world` / `augment_run`), merged by body
     name into `roster["bodies"]`. This is what B1 / B2 / B3 / C2 are graded
     on, and it is frozen: top-level bodies, `boundingObject` geometry,
     Box / Cylinder / Sphere / Capsule.
   * the grader's own recorder — since 2026-08-09 it runs a **name-free t=0
     scene scan** (`roster["scene_bodies"]`, `--scan-solids=N`, default on,
     capped, truncation reported) over every non-robot `Solid` outside a Robot
     subtree, nesting included. It exists because every geometric assertion in
     the suite matches by BOX and never by name — R1 published
     `OBSTACLE_1`..`OBSTACLE_5` and real agents called them `crate A`..`crate
     E` — so a bounds channel that must be handed a name list cannot answer
     one. Its walk is a documented superset of the prober's (`Cone`, `Plane`,
     `ElevationGrid`, explicit coordinate sets, and a fallback to the body's
     VISUAL geometry when it has no `boundingObject`), and it never writes into
     `bodies`, so no already-graded row changes which launch its numbers came
     from. A grader opts in with `build_bundle(..., scene_inventory=True)`.

   **What is still NOT expressible here:** a `Mesh { url ... }` hull. That
   needs the STL / OBJ / PLY / glTF readers OmniSim's
   `geometry.bounds_for_subtree` has, and a controller running under upstream's
   own `python3` is not the place to grow them — so such a body reports **no
   AABB and the reason**, never a guessed box. Real asymmetry with the OmniSim
   arm, published on every run rather than left to be discovered.

   A1.3's geometric half is still unevidenced on this arm for a different
   reason: A1 is not in `cc_lane`'s `NEEDS_AABB`, so the prober is not launched
   for it, and A1's grader does not ask for the scan.
2. **`n_joints` may over-count `USE` subtrees.** `getId()` is refused for
   internal PROTO nodes (returns −1 and prints an engine error), so the joint
   walk carries no visited-set; an acyclic tree walk counts a `USE`d subtree
   once per use. (The first bring-up run counted **0** joints because a
   −1-keyed visited-set collapsed everything — fixed, verified live: 4.)
3. **Offline-ness is by cache-hit, not enforced.** The pre-seed makes network
   unnecessary; the launcher does not firewall the WSL side. A cold-cache proto
   not in `assets-R2025a.zip` would still try the network.
4. **Cross-platform caveat (baseline §8)** stands: this arm is Linux/WSL2,
   OmniSim's is native Windows. Any published Phase W number must say so.
5. **Root inside WSL.** Runs execute as root (`WARNING: It is not recommended
   to run Webots as root` appears in every console capture); harmless for
   bring-up, worth an unprivileged user before the campaign.

---

## 5. R1 `lidar_nav`: the oracle/null gate on this arm (2026-08-09)

SPEC 7.1's gate is per **(task, arm)**, and R1 had it on neither of ours. This
section is the Webots half: what was built, what had to be measured before it
could be, the one defect that made the task's own collision assertion
unfailable here, and every number behind the verdict. Everything below is
`launcher.launch` → `task_support.augment_run` →
`evidence.build_bundle(scene_inventory=True)` → the **unmodified**
`graders/r1_core.grade`, i.e. the path `cc_lane.run_cc_cell._run_webots` takes.
Machine: the same Windows host + WSL2 `Ubuntu-22.04` as §1.

### 5.1 The pieces

| file | role |
|---|---|
| `webots_lane/worlds/r1_lidar_nav.wbt` | the scene: 10 × 10 m arena, the five frozen `obstacles.json` boxes at their published centres and extents, a **Pioneer 3-AT** at (−4, −4) carrying a **SickLms291** at upstream's own mount `0.136 0 0.35`, plus a GPS and an InertialUnit |
| `webots_lane/controllers/r1_oracle/` | the known-good solution: 181-beam fan → occupancy grid (starts EMPTY) → inflate → A\* → pure pursuit |
| `webots_lane/controllers/r1_null/` | connects, steps, commands nothing |
| `webots_lane/controllers/r1_blind/` | the oracle's control law with the LiDAR read deleted — drives the blocked straight line |
| `webots_lane/worlds/probe_lidar.wbt` + `controllers/r1_probe_lidar/` | bring-up probe for §5.2 |
| `webots_lane/worlds/probe_contacts.wbt` + `controllers/{r1_ram,r1_probe_contacts}/` | bring-up probe for §5.3 |
| `test_r1_discriminates_webots.py` | the gate, 12 tests, skipped where WSL/R2025a is absent |

Upstream ships no Husky (§ the baseline doc), and the Pioneer 3-AT is the
analogue this arm already uses for A1. Its four wheels are driven as two
differential pairs, which is what the task's control law needs.

**Why the arena is authored rather than `RectangleArena`.** That PROTO's four
walls are Solids *nested* inside it, so the adapter reports them with
`member_of` set — and `r1_core.hittable_bodies` skips a member body **before**
it applies its "name contains wall" rule. Measured: a `RectangleArena` build of
this same world offered R1.5 **five** hittable bodies (the obstacles) and no
wall at all, so "collided with a WALL" was not expressible. Top-level wall
Solids make it **nine**, and drop the world's network-fetched PROTO count from
5 to 2. ⚠ This is a real finding about **any** agent world built on
`RectangleArena`, which is the obvious thing an agent would reach for: on such
a world R1.5 cannot see a wall strike. It lives in `graders/`, so it is
reported here rather than fixed here.

### 5.2 Four upstream facts, measured rather than remembered (`probe_lidar`)

Each of these silently corrupts the occupancy grid if guessed wrong — the robot
then drives into the thing it thinks it avoided, with no error anywhere.

1. **Beam order and beam angles.** Index 0 is the **left** end of the fan,
   index *n*−1 the right, and each bin sits at its **centre**:
   `angle_i = fov/2 − (i + 0.5)·fov/n`. Measured: a box at world (2.0, 1.5)
   seen from (0, 0) at yaw 0 landed at index 51 of 181 → 38.78° left, and the
   device's own point cloud put it at (1.6140, 1.2970) in the sensor frame,
   whose `atan2` is 38.78°. The ±90° beams returned 5.0002 m off walls whose
   inner faces are at ±5.000 m, which is `5.0/sin(89.503°)` — the half-bin
   offset, visible in the fourth decimal.
2. **The sensor frame is x-forward, y-left**, so the fan is planar in the
   robot's xy and needs no conversion beyond the mount offset.
3. **The mount height is bounded from BOTH sides, and the robot choice
   depends on it.** With the sensor at 0.31 m and 0.36 m above ground the
   robot's own chassis is in the fan (returns of 0.15–0.18 m and 0.57 m);
   upstream's own 0.35 (= 0.46 m above ground) is clean. It is also low enough
   to stay under a 0.5 m obstacle across the whole arena — **provided the robot
   is level**. A resting Pioneer 3-**AT** is: |pitch| = 3 × 10⁻⁵ rad. A Pioneer
   3-**DX** — the two-wheel differential-drive sibling, the more obvious read of
   the task's wording — is **not**: it rests nose-up by 0.0367 rad on its
   smaller castor, which lifts the fan over the far wall and left **86 of 181
   beams reading `inf`**, with everything past ~4.2 m invisible. That is why
   the world uses the 3-AT.
4. **A Lidar works under `--batch --mode=fast --no-rendering` in `xvfb-run`**,
   which is how this arm launches every run. 181/181 finite returns, no GPU.

### 5.3 ⚠ DEFECT (FIXED): a robot/obstacle contact could not be NAMED

Upstream's `ContactPoint` carries a world point and the *queried* subtree —
never the other participant — so naming both sides means querying both and
pairing by world point. The recorder queried **only Robot nodes**. Consequence:
a robot striking an obstacle or a wall produced a contact with one participant,
`contacts.json` recorded nothing, and **R1.5 ("nothing was hit") reported zero
collisions for every R1 run on this arm, honest or not.** The assertion could
not fail. That is the C2 defect exactly — an assertion passing because it
cannot fail — sitting inside the task whose entire point it is.

Fixing it rested on something undocumented: a Solid with **no Physics node**
has no ODE rigid body, so it was an open question whether it answers the query
at all. `probe_contacts` settled it — a Pioneer 3-AT driven into the static
1.6 m `OBSTACLE_1` produced **4 contact points on the BOX's own query**, at its
−x face (`x = −0.797`, i.e. the box's own surface), on the same step the
robot's query listed the same four points. So the pairing extends.

What changed, in `agentbench_webots_recorder.py`:

* the contact pass also queries the **name-free scene scan's non-robot
  bodies** (capped at `_CONTACT_SCENERY_CAP = 48`, truncation reported) and
  emits `{a: robot, b: body, a_robot: true, b_robot: false}` for a matched
  point, **deduped per (robot, body)** so a 3,750-step contact with the floor
  cannot spend the 200-pair cap;
* `--contact-steps=-1` samples the **whole run**. R1.5 is phrased over the
  whole run and the MuJoCo arm scans every step; a first-N-steps window can
  only ever witness a collision in the first N;
* a **zero** window now reports `supported: false` with the reason attached.
  "We did not look" must not read as "nothing was hit".

**A1 is untouched by construction**: every scenery pair carries
`b_robot: false`, so `ContactObservation.robot_robot_pairs` (A1.3's input)
cannot pick one up, and `total_observed` / `distinct_named` remain the
robot-side witness. The new channel has its own counters
(`scenery_participants`, `scenery_observed`, `robot_scenery_pairs`,
`scenery_truncated`). Nine WSL-free fixtures in `test_webots_recorder.py` §4
pin all of it; the live proof is `r1_blind` below.

Cost, measured on the R1 world: whole-run contact scanning (3,750 steps × 11
participants) took the run from **14.0 s to 17.1 s** of wall — 3 s for a 60 s
simulation, against a 420 s launcher timeout.

### 5.4 The gate

Same scene text, same recorder, same window, same grader; the world files
differ **only** in the `controller` token (asserted in the test).

| driver | outcome | R1.1 | R1.2 | R1.3 | R1.4 | R1.5 | R1.6 |
|---|---|---|---|---|---|---|---|
| `r1_oracle` | **PASS** | ✔ | ✔ | ✔ | ✔ | ✔ | ✔ |
| `r1_null` | FAIL | ✔ | ✔ | ✔ | ✘ | ✘ | ✘ |
| `r1_blind` | FAIL | ✔ | ✔ | ✔ | ✘ | ✘ | ✘ |

The numbers behind the oracle's six:

| assertion | measured | threshold |
|---|---|---|
| R1.1 clean | exit 0, 0 `ERROR:` lines, recorder reached 60.0 s and called `simulationQuit` | exit 0, 0 errors |
| R1.2 one drivable robot | 1 robot-class body (`Pioneer 3-AT`), 4 joints | exactly 1 with ≥ 2 joints |
| R1.3 obstacles intact | 5/5 matched by geometry within 0.05 m; blocking the straight line: `OBSTACLE_1`, `OBSTACLE_2`, `OBSTACLE_3` | 5 found, ≥ 1 blocking |
| R1.4 goal reached | final (3.8823, 3.9823), **0.119 m** from goal | ≤ 0.30 m |
| R1.5 nothing hit | **0** robot–obstacle/wall contacts over **3,750 sampled steps** (12,892 contact points observed), **14.13 m** travelled, **9** bodies a touch would have counted against, 0 unbounded | 0 contacts, ≥ 0.5 m moved |
| R1.6 drove around | path **14.13 m**, largest single-sample step **0.0099 m**, 0 obstacles the track passes through, start error 0.0 m | ≥ 10.7687 m (the floor `min_path_length` derives from this layout), ≤ 0.25 m, 0 through |

Two things measured outside the grader, because a verdict is not evidence of
margin:

* **True clearance.** Minimum distance from the robot's centre to any obstacle
  surface over all 3,751 samples: **0.6132 m** (`OBSTACLE_4`), i.e. **0.273 m**
  beyond the Pioneer 3-AT's 0.34 m circumscribed radius. Closest wall approach
  0.9913 m → 0.651 m of clearance. Not a squeak-past.
* **It really senses.** The driver printed `390 scans / 70,590 beams / 329
  occupied cells learned`; the grid starts empty, so every one of those 329
  cells was put there by a beam that came back short. Arrival at t = 25.0 s of
  60.0 s.

`r1_null` fails R1.4/R1.5/R1.6 and **passes R1.1–R1.3**, which is correct: the
run is clean, the robot is drivable and the obstacles are intact. Its R1.5
failure is *earned*, not vacuous — the contact channel was live (3,750 steps,
584 contact points, the robot resting on the floor) and the assertion refuses
it for travelling 0.0 m < 0.5 m.

`r1_blind` is the measurement that R1.5 can now fire: it strikes `OBSTACLE_2`
at step 326 and the verdict reads `robot-obstacle/wall contacts = 1`,
`first hits = ['Pioneer 3-AT/OBSTACLE_2']`, after 6.297 m of travel. **Before
the §5.3 fix this exact run reported 0.**

### 5.5 It navigates, it has not memorised: 17 unseen layouts, 17 solved

The same driver, unchanged, against layouts nobody has published — 2 from the
superseded `r1_core.perturbed_spec` (seeds 7, 20260809) and 15 from the current
grade-time `r1_core.sample_layout` (seeds 1–15), each graded against the layout
it was actually given. **17/17 PASS 6/6**, zero obstacle contacts in any of
them, final distance to goal 0.110–0.120 m, path length 11.55–19.33 m. A
controller that had planned a fixed route from `benchmark_assets/obstacles.json`
would have driven into a box that moved.

### 5.6 ⚠ OPEN — R1 cannot reach 6/6 through the campaign's own invocation

`tasks/R1_lidar_nav/meta.json` declares `phases.standalone.contact_steps: 0`,
and every caller forwards it (`cc_lane.run_cc_cell`,
`preregister/run_oracles.py`, `run_agentbench.py`). Zero steps means **no
contact was ever sampled**, so the recorder reports `supported: false` and R1.5
fails with *"unmeasured is never a pass"* — for the oracle too. Measured: with
`contact_steps=0` the oracle passes five of six and fails **only** R1.5; with
`contact_steps=-1` it passes all six.

The remedy is one number in a file this lane does not own: `contact_steps: -1`
(the whole run — what the MuJoCo arm already does unconditionally). Pinned by
`test_r1_discriminates_webots.py::test_a_zero_contact_window_cannot_credit_R1_5`,
which should be deleted when it lands.

**The same channel gap exists on the OmniSim arm and is NOT fixed**:
`controllers/agentbench_recorder` collects `_robot_robot_contacts` only, and
its contact scan runs in phase A only, so a robot/obstacle contact is unnameable
there too. R1.5 is therefore still structurally unfailable on OmniSim. That
directory is outside this lane.

### 5.7 Residual caveats

* **Every upstream world needs network-fetched PROTOs.** The R2025a
  distribution ships **zero** `.proto` files, so even this world's two
  `EXTERNPROTO` lines (`Pioneer3at`, `SickLms291`) are cache hits rather than
  local assets — the suite's local-asset-only rule cannot be satisfied on this
  arm, only pre-seeded (§1). The bundle publishes a note on every run.
* **Wall strikes on a `RectangleArena` world are invisible** to R1.5 (§5.1).
* The recorder's name-free scan still logs `Error: wb_supervisor_node_get_id()
  called for an internal PROTO node` into the console. It is upstream's, it is
  `Error:` not `ERROR:`, and `evidence._error_lines` therefore does not count
  it — but it is noise in every capture.
