# DEMOS.md — the demo dictionary

Every user-facing demo in this repo. One row per demo with everything you need to find it, run it, and understand what it shows.

For the equivalent map keyed by `.wbt` file, see [WORLDS.md](WORLDS.md). For the repo layout overview, see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## Browse demos visually — the in-sim launcher

For interactive browsing inside OmniSim itself, run `launch.bat` with no arguments. The default world is the **OmniSim demo launcher** ([`projects/samples/demos/worlds/omnilink_launcher.wbt`](projects/samples/demos/worlds/omnilink_launcher.wbt)): a small stage with a floating orb robot. Right-click the orb → *Show Robot Window* → a side-panel gallery shows every demo grouped by category, with search and one-click *Launch*. Catalogue: [`projects/samples/demos/controllers/omnilink_launcher/demos.json`](projects/samples/demos/controllers/omnilink_launcher/demos.json) (edit this AND a row in this file when adding a new demo).

## Quick picks

| Goal | Demo |
|---|---|
| Hello world | [Warehouse Husky](#warehouse-husky-onboarding) |
| Quadruped walking (RL deploy) | [Spot / Go2 / B2](#quadruped-locomotion-rl-deploy) |
| Humanoid stand & walk (RL deploy) | [G1 / H1 / Valkyrie](#humanoid-rl-deploy) |
| G1 picks a real box, carries it, turns a corner | [Box delivery — the BATON headliner](#g1-skill-demos--baton-sequences) |
| G1 climbs stairs *(legs-only, no vertical assist)* | [Stair climb + summit stand](#g1-skill-demos--baton-sequences) |
| Browse / run / compose every G1 skill | [The Skill Library](#the-skill-library--one-cli-for-all-of-the-above) |
| Type to talk to a robot | [Chat demos](#1-chat-demos--single-robot-natural-language-console) |
| Multi-robot agent orchestration | [Warehouse Foreman](#warehouse-foreman--multi-agent-orchestration) |
| Long-running memory dividend | [Patrol Squad](#patrol-squad--cross-session-memory) |
| Drone autonomy | [Drone Surveyor](#drone-surveyor--perimeter-survey--marker-detection) |
| City traffic — follow a car | [The Living City](#7-misc--showcase) |

---

## Onboarding starter

### Warehouse Husky *(onboarding)*

The default demo. Supervisor-enabled Husky random-walks a 30 × 18 m warehouse with reactive collision recovery. Shows the URDF importer, supervisor API, motor torque pipeline, and camera follow.

| | |
|---|---|
| World | [`projects/samples/demos/worlds/showcase/warehouse_husky.wbt`](projects/samples/demos/worlds/showcase/warehouse_husky.wbt) |
| Controller | [`husky_random`](projects/default/controllers/husky_random/) |
| Launch | `launch.bat` *(no args)* |

---

## 1. Chat demos — single robot, natural-language console

Right-click the robot → *Show Robot Window* → a chat side panel opens. Type `home`, `wave hello`, `forward 1 m`, `turn left 90 degrees`, `stop`. Offline = regex intent router. Set `OMNI_KEY` for full LLM routing through OmniLink.

Full guide: [`docs/guide/omnilink-chat-demos.md`](docs/guide/omnilink-chat-demos.md). Index: [`projects/samples/demos/worlds/chat/OMNILINK_CHAT_DEMOS.md`](projects/samples/demos/worlds/chat/OMNILINK_CHAT_DEMOS.md).

### Arms

All three drive over the [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) HTTP surface
(§4 of [AGENTS.md](AGENTS.md)). Verified driving: joint tracking to ~0.02–0.03 rad, and damped-least-squares IK
reaching a Cartesian target to ~4.6 cm. Position control and suction / parallel-jaw picking — **no force control and
no in-hand manipulation**.

| Demo | World | Bridge controller |
|---|---|---|
| Universal Robots UR5e *(with IK)* | [`omnilink_ur5e.wbt`](projects/samples/demos/worlds/chat/omnilink_ur5e.wbt) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) |
| Universal Robots UR3e | [`omnilink_ur3e.wbt`](projects/samples/demos/worlds/chat/omnilink_ur3e.wbt) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) |
| Universal Robots UR10e | [`omnilink_ur10e.wbt`](projects/samples/demos/worlds/chat/omnilink_ur10e.wbt) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) |
| Three UR5e arms *(one console each)* | [`omnilink_multi_arm.wbt`](projects/samples/demos/worlds/chat/omnilink_multi_arm.wbt) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) ×3 (ports 8765–8767) |

### Mobile bases

| Demo | World | Bridge controller |
|---|---|---|
| Clearpath Husky | [`omnilink_husky.wbt`](projects/samples/demos/worlds/chat/omnilink_husky.wbt) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| Clearpath Jackal | [`omnilink_jackal.wbt`](projects/samples/demos/worlds/chat/omnilink_jackal.wbt) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| Husarion Rosbot | [`omnilink_rosbot.wbt`](projects/samples/demos/worlds/chat/omnilink_rosbot.wbt) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| Husarion Rosbot XL | [`omnilink_rosbot_xl.wbt`](projects/samples/demos/worlds/chat/omnilink_rosbot_xl.wbt) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| TurtleBot3 Burger | [`omnilink_tb3_burger.wbt`](projects/samples/demos/worlds/chat/omnilink_tb3_burger.wbt) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| TurtleBot3 Waffle | [`omnilink_tb3_waffle.wbt`](projects/samples/demos/worlds/chat/omnilink_tb3_waffle.wbt) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| TurtleBot3 Waffle Pi | [`omnilink_tb3_waffle_pi.wbt`](projects/samples/demos/worlds/chat/omnilink_tb3_waffle_pi.wbt) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |

### Quadruped

| Demo | World | Bridge controller |
|---|---|---|
| Boston Dynamics Spot *(poses only)* | [`omnilink_spot.wbt`](projects/samples/demos/worlds/chat/omnilink_spot.wbt) | [`omnilink_quadruped_bridge`](projects/samples/demos/controllers/omnilink_quadruped_bridge/) |

### Aerial

| Demo | World | Bridge controller |
|---|---|---|
| DJI Mavic 2 Pro *(+ Drone Surveyor flagship)* | [`omnilink_mavic.wbt`](projects/samples/demos/worlds/chat/omnilink_mavic.wbt) | [`mavic_omnilink_bridge`](projects/samples/demos/controllers/mavic_omnilink_bridge/) |

---

## 2. Flagship demos (OmniSim engine showcases)

These showcase **OmniSim's own engine** — the URDF importer, the Newton/MuJoCo physics deploy, the RL pipeline, and the manipulation stack. **No `OMNI_KEY` / LLM needed.** (The OmniLink agent demos that used to be the flagship set now live in [§3](#3-omnilink-agent-demos-agent-layer-on-top-of-omnisim).)

**Locomotion + Humanoid (Tiers B/C)** — these are **RL deploy** worlds that need a set of Newton env vars + a policy `.onnx`, so they launch via their `scripts/dev/run_*deploy*.ps1` script (which sets that env), **not** a bare world load. Add `-Gui` to watch live.

> **Honest status is canonical.** Every one-liner below is sourced from [`docs/developer/rl-current-state.md`](docs/developer/rl-current-state.md) — the single source of truth for OmniSim RL. "Stands" ≠ "stands via RL"; "walks" ≠ "walks durably". Read it before quoting any robot result.

### Quadruped locomotion (RL deploy)

RL-deploy walking / recovery under OmniSim Newton (MuJoCo solver). Launch each via its script; add `-Gui` to watch live.

| Demo | Launch | Status (per rl-current-state.md) |
|---|---|---|
| Spot velocity-conditioned walk / stop / walk | `scripts/dev/run_spot_walk_vc_deploy.ps1` | ✅ Newton-verified — **47.8 m, 0 falls**, 0.32 m/s, bz 0.553 (`gpu_spot_walk_vc_main`); one policy walks, decelerates, stands, resumes |
| Spot straight walk (G1 foot-space recipe) | `scripts/dev/run_spot_walk_deploy.ps1` | ✅ walks dead straight — the learned residual is **load-bearing** (+44 m with policy vs +0.13 m bare; verified 2026-06-26) |
| Go2 walk | `scripts/dev/run_go2_walk_deploy.ps1` | ✅ +86 m, 0 falls, ~0.38 m/s under Newton |
| Spot crouch–recover | `scripts/dev/run_spot_crouch_deploy.ps1` | ✅ quasi-static; replays a certified ghost open-loop, 0 falls |
| Spot get-up from the ground | `scripts/dev/run_spot_getup_deploy.ps1` | ✅ RL rise → faded hold, 0 falls |
| B2 get-up | `scripts/dev/run_b2_getup_deploy.ps1` | ✅ rise solved — ⚠️ the post-rise *hold* is chaotically fragile |
| B2 walk | `scripts/dev/run_b2_walk_deploy.ps1` | ⚠️ trot-model + RL-policy deploy harness present (bare-model baseline ~0.22 m/s); run it to see current forward progress |
| Spot jump | `scripts/dev/run_spot_jump_deploy.ps1` | ⚠️ experimental deploy harness |
| Spot / B2 hill walk | `scripts/dev/run_spot_hill_deploy.ps1`, `run_b2_hill_deploy.ps1` | ⚠️ ghost pipeline done + owner-approved — **BLOCKED** at the flat→ramp transition + slope roll-instability (~2.3 m cap) |

### Humanoid (RL deploy)

Humanoid walking + standing under OmniSim Newton. The flagship result is **OmniSim's own in-engine G1 walk, trained by Shadowing** (train == deploy bit-exact) — and it is the canonical way OmniSim makes legged-robot policies. This **supersedes** the 2026-06 "from-scratch G1 doesn't walk" finding. The **re-hosted Unitree policies** and their behavior-clones (BC) also walk durably (G1/H1). Deterministic **stands** hold for H1 and Valkyrie; the G1 needs its **active-balance (RL) stand** (`g1_hstand_deploy`), not the deterministic pose. Atlas remains unsolved research. Full canonical status: [rl-current-state.md](docs/developer/rl-current-state.md).

> ⚠️ **DISCLOSURE — THE PUPPET RIG. The flagship G1 walk is NOT a free-standing walk.** Every row and BATON sequence marked **🪝 craned** below runs on a **weight-bearing balance harness**: `HARNESS_LAM0=0.9`, `HARNESS_KZ=2000`, up to **700 N** of upward pelvis force — roughly **2× the 34 kg G1's body weight** — plus **±350 N·m** of attitude torque (the `_FCAP`/`_TCAP` clamps in [`g1_walk_recipe.py`](projects/policies/training/g1_walk_recipe.py)). The code's own comment is explicit: *"a fresh net doesn't tip over — it BUCKLES straight down… **the toddler harness holds weight, not just tilt**."* The robot is fully physical and does real footwork, but under 🪝 it is **not carrying its own weight unaided**.
>
> The **honest exception is the stair climb** (`run_climb_stairs.sh` / `run_climb_stairs_stand.sh`), which sets **`HARNESS_KZ=0`** — no vertical wire at all, so **the legs genuinely do the lifting**; the crane only trims lateral/attitude. That result stands on its own legs, literally.

| Demo | Launch | Status (per rl-current-state.md, 2026-07-04; flagship row 2026-07-06) |
|---|---|---|
| ⭐ **THE DECENT WALKER — FLAGSHIP humanoid demo (owner-designated 2026-07-06)** | [`projects/policies/worlds/run_g1_decent_walker.ps1`](projects/policies/worlds/run_g1_decent_walker.ps1) *(script only — the demo needs the full deploy env; a bare world load shows a lifeless robot)* | ✅ **owner-verified live** ("a decent looking walking robot for the first time") — **🪝 craned** (`LAM0=0.9`, `KZ=2000`; see the disclosure above): G1 walks the OFFICIAL Unitree gait on the visible-harness PUPPET rig beside its ghost hologram; LSTM+foresight champion [`projects/policies/training/runs/wr_decent_walker.pt`](projects/policies/training/runs/), **WBMATCH4 0.868** (honest shape-only ruler, exam-verified K=2048); natural thigh-clearing arm swing (ghost v3). Known-open: live pace below trainer (stride-gap thread). |
| **G1 walk (OmniSim Shadowing — canonical method)** | via [`projects/policies/training/`](projects/policies/training/README.md): `run_walk_rl.sh` deploy on [`g1_walk_ghost2.wbt`](projects/policies/worlds/g1_walk_ghost2.wbt) (env-driven; **VERIFY-BEFORE-SHOW** via [`verify_walkstop.py`](projects/policies/training/verify_walkstop.py)) | ⚠️ **in-engine, train==deploy — but NOT unconditionally durable, and 🪝 craned.** Durability champion `projects/policies/training/runs/wr_showpiece.pt` 45.6 m / 101 s / 0 falls; style champion `projects/policies/training/runs/wr_calm_champion.pt` WBMATCH 0.908 vs the owner-approved reference. The walk skill's own manifest ([`skills/humanoid/walk/skill.json`](projects/policies/skills/humanoid/walk/skill.json)) is the honest word: *"Long straights have a residual live fall rate (~open durability)."* Live pace ~0.2 m/s. |
| **G1 stair climb (walking-ghost + terrain)** | [`projects/policies/demos/run_climb_stairs.sh`](projects/policies/demos/run_climb_stairs.sh) on [`g1_climb_stairs_demo3.wbt`](projects/policies/worlds/g1_climb_stairs_demo3.wbt) | ✅ **full 5-step live climb @ 3 cm risers** (shipped 2026-07-08) — real foot steps, **legs-only: `HARNESS_KZ=0`**, no vertical assist (base z 0.72→0.88). ⚠️ **"stairs" are NOT solved: 3 cm is the MEASURED CEILING** for the stock-foot G1 climbing legs-only — **4 cm ≈ 2 steps, 5 cm ≈ 0** (the small-foot propulsion wall). Real staircases (~17 cm) need a bigfoot morphology or a vertical assist. Gait is also not style-clean (heading wanders up to ~124° mid-climb). |
| **G1 walk (Unitree re-host)** | `projects/policies/worlds/run_g1_unitree_walk.ps1` | ✅ **33.7 m, 0 falls, 0.48 m/s** (Unitree `motion.pt`); the team's BC clone `g1_bc_walk.pt` walks 44 m+ likewise |
| **H1 walk (Unitree re-host)** | `projects/policies/worlds/run_h1_unitree_walk.ps1` | ✅ **30 m, 0 falls, 0.42 m/s** (Unitree `motion.pt`); BC clone `h1_bc_walk.pt` walks 26 m+ likewise |
| **G1 stand (active-balance RL)** | [`projects/policies/worlds/g1_hstand_deploy.wbt`](projects/policies/worlds/g1_hstand_deploy.wbt) | ✅ holds via whole-body active balance where the deterministic G1 pose tips. **Deploy** figure (the `stand_rl` skill manifest, measured in the box-delivery sequence): dead-still hold — **vx 0.00, roll 0.001, z 0.778, 0 falls across all BATON switches**. *(The often-quoted "surv 1.0" is a **trainer** metric, not a deploy result — don't cite it as one.)* |
| **H1 stand** | `scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1` | ✅ holds (bz 0.977, 0 falls) — deterministic pure pose |
| **Valkyrie stand** | `scripts/dev/run_humanoid_stand_deploy.ps1 -Robot valkyrie` | ✅ holds (bz 1.104, 0 falls) — stiff-ankle pure pose (130 kg) |
| H1 / Valkyrie shadow-walk | `scripts/dev/run_humanoid_walk_deploy.ps1 -Robot h1\|valkyrie` | ⚠️ NOT durable — feasible walking shadow tracked with stiff PD; falls ~2 s; RL tracker pending (distinct from the working Unitree H1 walk above) |
| G1 deterministic stand *(superseded)* | `projects/policies/research/runners/run_g1_stand_deploy.ps1` | ❌ deterministic pure pose tips forward, FALL@~1.38 s — use the **active-balance stand above** instead |
| Pre-Shadowing G1 from-scratch *(archived)* | [`projects/policies/research/runners/`](projects/policies/research/runners/) | ❌ historical: from-scratch face-planted ~1.3 s, ghost-built v6 ~1.44 s — **superseded by the Shadowing walk above** |
| Atlas stand *(archived)* | [`projects/policies/research/`](projects/policies/research/) | ❌ negative result — PPO ≈ zero-action baseline (~0.8 s tip); never Newton-deployed |

### G1 skill demos + BATON sequences

The **composed** G1 demos: a walk policy hands the baton to a carry / turn / stand specialist mid-run (**BATON** = policy switching at gated handovers). Each is a one-line **launcher script** — they set a large deploy env (engine pymod hook + ghost LUTs + corridors + harness + the BATON course), so **a bare world load will NOT work**; it shows a lifeless robot. Always launch via the script:

```bash
bash projects/policies/demos/run_box_delivery.sh        # [dur] [gui|headless]
```

🪝 = runs on the **weight-bearing puppet harness** — see the [disclosure above](#humanoid-rl-deploy). **Legs-only** = `HARNESS_KZ=0`, no vertical assist.

| Demo | Launch (from repo root) | Status |
|---|---|---|
| ⭐ **Box delivery — the BATON headliner** | [`bash projects/policies/demos/run_box_delivery.sh`](projects/policies/demos/run_box_delivery.sh) | ✅ **0 falls.** G1 walks to cart A, **picks a real 1.5 kg box** (physical body, proximity-gated two-phase lift — it can never levitate), carries it down the corridor to cart B, **sets it down** on real contact, walks on, takes a **real ~90° footwork corner** (TURN-LOOP, `wtz=0`), walks away, ends in a stand. 🪝 **craned**. ⚠️ The corner is only certified in the **0→90° heading band** (10/10 there; every sweep past ~95° fell or spun) — the full there-and-back shuttle needs a heading-randomized retrain. ⚠️ BATON's *"switching beats a monolith"* is an **OPEN HYPOTHESIS**, not a measured result. |
| **Box delivery — classic** *(the verified baseline)* | [`bash projects/policies/demos/run_box_delivery_classic.sh`](projects/policies/demos/run_box_delivery_classic.sh) | ✅ **verified, 0 falls** — walk → stand → pick the real 1.5 kg box → carry → place (real contact) → walk on → stand. **No corner** (crane-yaw steering). 🪝 **craned**. |
| **Walk → turn → walk** | [`bash projects/policies/demos/run_walk_turn_walk.sh`](projects/policies/demos/run_walk_turn_walk.sh) | ✅ **3/3, 0 falls** (SOLVED 2026-07-10, `72a7bb19`). Walks ~5.2 m along +x, turns a **real ~90° by footwork** (`wtz=0`, no rope — lands **90.6–95.6° actual** in 2–3 TURN-LOOP passes), then walks a **clean straight leg on the new heading**. 🪝 **craned**. |
| **Turn solo** | [`bash projects/policies/demos/run_turn_solo.sh`](projects/policies/demos/run_turn_solo.sh) | ⚠️ **experimental** (per the skill registry). The footwork-turn policy **by itself**: settle to a stand → ~90° turn on pure footwork (crane-yaw auto-OFF, `wtz=0`) → decel → hold. Most reliable from a clean stand; the in-sequence variance lives in the walk→turn handoff. 🪝 **craned**. |
| ⭐ **Stair climb + summit stand** | [`bash projects/policies/demos/run_climb_stairs_stand.sh`](projects/policies/demos/run_climb_stairs_stand.sh) | ✅ Climbs the **full 5 treads (3 cm risers)** **legs-only (`HARNESS_KZ=0`)**, then a **position-gated BATON handover** swaps in the stand specialist and the robot **holds a verified motionless stand on the top landing** (FOOT_LOG: base 0.03→2.72 m, z 0.72→0.877; final seconds — both feet past the landing line at summit height, base x-std ~1 mm; the certifier [`certify_stair_human.py`](projects/policies/training/certify_stair_human.py) gates at **< 5 mm over the final 8 s**). ⚠️ Same **3 cm ceiling** as the climb below; the climb itself is **not style-clean** (heading wanders to ~124°). |
| **Stair climb** *(5 treads @ 3 cm)* | [`bash projects/policies/demos/run_climb_stairs.sh`](projects/policies/demos/run_climb_stairs.sh) | ✅ / ⚠️ — see the [G1 stair-climb row above](#humanoid-rl-deploy). **Legs-only**; **3 cm is the measured ceiling** (4 cm ≈ 2 steps, 5 cm ≈ 0). |
| **Book grasp** *(physics arm grasp)* | [`bash projects/policies/demos/run_book_grasp.sh`](projects/policies/demos/run_book_grasp.sh) | ⚠️ **EXPERIMENTAL / BLOCKED** (the script's own header says so). The G1 picks up a **real Newton box with its arms** (zero kinematic writes). Blocked at a **warp contact-kernel** defect near the cart — runs die within ~40–90 ticks of reach onset. The **shipping** pick-and-place demo is `run_box_delivery.sh` (hand-tracked grab). |

### The Skill Library — one CLI for all of the above

Every skill above is packaged as a **versioned manifest** binding its ghost + validator verdict + deploy env + champion checkpoint + provenance, so the same trainer and deploy stack reproduce it. **10 skills + 4 BATON sequences** behind one CLI:

```bash
python projects/policies/skills/skill_lib.py list                    # the catalogue (skills + sequences, with honest status)
python projects/policies/skills/skill_lib.py sequence box_delivery   # run a BATON demo
python projects/policies/skills/skill_lib.py preview walk            # just the ghost hologram
python projects/policies/skills/skill_lib.py show climb_stairs       # ghost, checkpoint, deploy env, provenance
python projects/policies/skills/skill_lib.py verify-demos            # proves the manifests reproduce the demo scripts
```

Skills span robots (G1 / H1 / Go2 / Spot) and methods (Shadowing / Unitree re-host / deterministic overlay), each carrying its own `verified` / `experimental` status — `climb_stairs` and `turn_in_place` are **experimental**, and the CLI says so. Full reference: [`docs/developer/skill-library.md`](docs/developer/skill-library.md) · [`projects/policies/skills/README.md`](projects/policies/skills/README.md).

---

## 3. OmniLink agent demos (agent layer on top of OmniSim)

The full-fat **agent-driven** demos — each pairs a bespoke world with a production-grade OmniLink agent (set `OMNI_KEY` for full LLM routing; several also run offline). These used to be the flagship set; they now live here so the flagship slot can showcase the OmniSim engine itself. See [`agents/ROADMAP.md`](agents/ROADMAP.md) for the build roadmap.

### Warehouse Foreman — multi-agent orchestration

Two agents (Foreman / Picker), one operator command: *"Move two pallets of cardboard from rack row M to the loading dock."* The Husky pushes pallets to the dock; there is no arm leg in the mission.

| | |
|---|---|
| World | [`projects/samples/demos/worlds/flagship/warehouse_logistics.wbt`](projects/samples/demos/worlds/flagship/warehouse_logistics.wbt) |
| Agent | [`agents/production/warehouse_foreman/`](agents/production/warehouse_foreman/) (+ [`warehouse_picker`](agents/production/warehouse_picker/)) |
| Docs | [`agents/production/warehouse_foreman/docs/ARCHITECTURE.md`](agents/production/warehouse_foreman/docs/ARCHITECTURE.md), [`RESULTS.md`](agents/production/warehouse_foreman/docs/RESULTS.md) |
| Status | Shipped 2026-05-02 |

### Patrol Squad — cross-session memory

Two Huskies in the same warehouse, same `Patrol Husky` profile, sector-tagged, with cross-sweep change-detection from long-term memory.

| | |
|---|---|
| World | [`projects/samples/demos/worlds/flagship/warehouse_patrol.wbt`](projects/samples/demos/worlds/flagship/warehouse_patrol.wbt) |
| Agent | [`agents/production/warehouse_patrol/`](agents/production/warehouse_patrol/) |
| Docs | [`agents/production/warehouse_patrol/docs/RESULTS.md`](agents/production/warehouse_patrol/docs/RESULTS.md) |
| Status | Shipped 2026-05-02 |

### Drone Surveyor — perimeter survey + marker detection

Mavic 2 Pro autonomously surveys the warehouse perimeter, gimbal camera + agent vision narrate red markers, complete-mission report includes world positions.

| | |
|---|---|
| World | [`projects/samples/demos/worlds/chat/omnilink_mavic.wbt`](projects/samples/demos/worlds/chat/omnilink_mavic.wbt) *(merged with the Mavic chat demo — same world, two surfaces)* |
| Bridge | [`mavic_omnilink_bridge`](projects/samples/demos/controllers/mavic_omnilink_bridge/) |
| Agent | [`agents/production/drone_surveyor/`](agents/production/drone_surveyor/) |
| Docs | [`agents/production/drone_surveyor/docs/PLAN.md`](agents/production/drone_surveyor/docs/PLAN.md), [`RESULTS.md`](agents/production/drone_surveyor/docs/RESULTS.md) |
| Status | Iter 0–2 shipped; iter-3 LLM run pending |

### Husky Maze — vision-driven navigation

Single Husky across four mazes with progressively harder briefs. The reference OmniLink agent.

| Variant | World | What's hard | Who can solve |
|---|---|---|---|
| Simple | [`husky_maze.wbt`](projects/samples/demos/worlds/flagship/husky_maze.wbt) | Drive to (10, 0) | script or agent |
| Unknown | [`husky_maze_unknown.wbt`](projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt) | Map gated; lidar wall-follow | script or agent |
| Corners | [`husky_maze_corners.wbt`](projects/samples/demos/worlds/flagship/husky_maze_corners.wbt) | "Visit four corners and return" | **agent only** |
| Visual | [`husky_maze_visual.wbt`](projects/samples/demos/worlds/flagship/husky_maze_visual.wbt) | "Find the RED cylinder via camera" | **agent only** |
| Blind | [`husky_maze_blind.wbt`](projects/samples/demos/worlds/flagship/husky_maze_blind.wbt) | No map, no lidar; perception tags from sidecar | **agent only** |

Agent: [`agents/production/husky_maze/`](agents/production/husky_maze/). Docs: [`OVERVIEW.md`](agents/production/husky_maze/docs/OVERVIEW.md).

### Husky Swarm — multi-robot coordination

| | |
|---|---|
| World | [`projects/samples/demos/worlds/flagship/omnilink_husky_swarm.wbt`](projects/samples/demos/worlds/flagship/omnilink_husky_swarm.wbt) |
| Bridge | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) (multiplexed) |

---

## 4. Specialist agent templates *(starter kits)*

Profile-only OmniLink agents that drive existing bridges. Copy, edit `profile.json`, push via `register.py` — no Python process needed.

| Template | Targets | What it shows |
|---|---|---|
| [`omnisim_roomba`](agents/templates/roomba/) | Any wheeled base | Profile-only specialist for waypoint patrol |

See [`agents/templates/README.md`](agents/templates/README.md) for the pattern comparison.

---

## 5. Physics / CUDA showcases

| Demo | World | What it shows |
|---|---|---|
| CUDA particles smoke | [`cuda_particles_smoke_test.wbt`](projects/samples/demos/worlds/physics/cuda_particles_smoke_test.wbt) | CUDA particle pool resolves a regex-numbered particle pattern |
| Granular sand | [`granular_sand_demo.wbt`](projects/samples/demos/worlds/physics/granular_sand_demo.wbt) | Cohesive-particle sand interaction |
| Newton smoke | [`newton_smoke_test.wbt`](projects/samples/demos/worlds/physics/newton_smoke_test.wbt) | Newton physics engine baseline |
| Newton Husky | [`newton_husky_smoke_test.wbt`](projects/samples/demos/worlds/physics/newton_husky_smoke_test.wbt) | Husky + Newton |
| Newton Husky swarm | [`newton_husky_swarm_drive.wbt`](projects/samples/demos/worlds/physics/newton_husky_swarm_drive.wbt) | Multi-Husky + Newton |
| Newton Husky head-on (×3 variants) | [`newton_husky_head_on.wbt`](projects/robot_combat/worlds/tests/newton_husky_head_on.wbt), [`_2`](projects/robot_combat/worlds/tests/newton_husky_head_on_2.wbt), [`_damage`](projects/robot_combat/worlds/tests/newton_husky_head_on_damage.wbt) | Collision + damage — see [`projects/robot_combat/`](projects/robot_combat/) for the full combat dev environment |
| BattleBox proving ground | [`battlebox_husky_proving.wbt`](projects/robot_combat/battlebots/worlds/battlebox_husky_proving.wbt) | Two Huskies in the new [`BattleBox`](projects/robot_combat/battlebots/protos/BattleBox.proto) arena — killsaw, pushers, side screws |
| BattleBox duel (spinner vs wedge) | [`battlebox_duel.wbt`](projects/robot_combat/battlebots/worlds/battlebox_duel.wbt) | Phase-2 headliner — two `BattleBot` PROTOs with different weapons + match director + broadcast cuts |
| BattleBox royal rumble | [`battlebox_royal_rumble.wbt`](projects/robot_combat/battlebots/worlds/battlebox_royal_rumble.wbt) | All 5 weapon archetypes free-for-all — last bot standing |

---

## 6. Generated worlds

Procedurally generated scaffolds (omniworld + seeds), all browsable from the in-sim launcher's *Generated worlds* category. Detail per world: [WORLDS.md §2](WORLDS.md#2-generated-worlds).

| Demo | World |
|---|---|
| Flat ground | [`flat_ground.wbt`](distribution/generated_worlds/flat_ground.wbt) |
| Warehouse | [`warehouse.wbt`](distribution/generated_worlds/warehouse.wbt) |
| Urban block | [`urban_block.wbt`](distribution/generated_worlds/urban_block.wbt) |
| Outdoor desert | [`outdoor_desert.wbt`](distribution/generated_worlds/outdoor_desert.wbt) |
| Outdoor forest | [`outdoor_forest.wbt`](distribution/generated_worlds/outdoor_forest.wbt) |
| Indoor apartment | [`indoor_apartment.wbt`](distribution/generated_worlds/indoor_apartment.wbt) |
| Mars | [`mars.wbt`](distribution/generated_worlds/mars.wbt) |
| Mars — night | [`mars_night.wbt`](distribution/generated_worlds/mars_night.wbt) |
| Earth — night | [`earth_night.wbt`](distribution/generated_worlds/earth_night.wbt) |

---

## 7. Misc / showcase

| Demo | World | What it shows |
|---|---|---|
| The Living City | [`city_traffic.wbt`](projects/samples/demos/worlds/showcase/city_traffic.wbt) | Generator-driven 4×4 city — 48 cars routing with traffic signals, pedestrians on crossings, a city bus, day/night cycle. Follow a car: click it → right-click → *Follow Object* (or `F5`). Regenerate via [`gen_city_traffic.py`](scripts/dev/gen_city_traffic.py) |
| Desert ruins | [`desert_ruins.wbt`](projects/samples/demos/worlds/environments/desert_ruins.wbt) | Rough-terrain outdoor navigation |
| Husky rocks traverse | [`husky_rocks_traverse.wbt`](projects/samples/demos/worlds/showcase/husky_rocks_traverse.wbt) | Lunar/Mars-style traverse |

---

## 8. Sim-to-real bridges

| | |
|---|---|
| Arm bridge stub | [`agents/bridges/arm_bridge_stub.py`](agents/bridges/arm_bridge_stub.py) |
| Mobile bridge stub | [`agents/bridges/mobile_bridge_stub.py`](agents/bridges/mobile_bridge_stub.py) |
| Base class | [`agents/bridges/bridge_base.py`](agents/bridges/bridge_base.py) |
| Docs | [`agents/bridges/README.md`](agents/bridges/README.md) |

Same `/list_robots` / `/prompt` / `/tool` / `/get_robot_state` / `/stop_robot` HTTP surface as the sim bridges. Drop in a real robot driver and the same OmniLink agents work unchanged.

---

## How to add a new demo

See the recipe in [ARCHITECTURE.md](ARCHITECTURE.md#how-to-add-a-new-demo).
