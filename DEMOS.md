# DEMOS.md — the demo dictionary

Every user-facing demo in this repo. One row per demo with everything you need to find it, run it, and understand what it shows.

For the equivalent map keyed by `.wbt` file, see [WORLDS.md](WORLDS.md). For the repo layout overview, see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## Browse demos visually — the in-sim launcher

For interactive browsing inside OmniSim itself, run `launch.bat` with no arguments. The default world is the **OmniSim demo launcher** ([`projects/samples/demos/worlds/omnilink_launcher.omniworld`](projects/samples/demos/worlds/omnilink_launcher.omniworld)): the "hello world" house scene — a small sunlit house lot (the Beauty Bench, wgpu-rendered) with a floating orb robot; press V to reveal the draggable sun marker. Right-click the orb → *Show Robot Window* → a side-panel gallery shows every demo grouped by category, with search and one-click *Launch*. Catalogue: [`projects/samples/demos/controllers/omnilink_launcher/demos.json`](projects/samples/demos/controllers/omnilink_launcher/demos.json) (edit this AND a row in this file when adding a new demo).

## Quick picks

| Goal | Demo |
|---|---|
| Hello world | [Warehouse Husky](#warehouse-husky-onboarding) |
| Hold a part with a real friction grip | [`omniarm6_real_pick_place.omniworld`](projects/samples/demos/worlds/flagship/omniarm6_real_pick_place.omniworld) — friction only, nothing welded, and it ships a drop control (`PICK_CONTROL_DROP=1` must leave the block behind). Run with `--duration 45`. Guide: [docs/guide/friction-grasp.md](docs/guide/friction-grasp.md) |
| Quadruped walking (RL deploy) | [OmniQuad / Go2 / B2](#quadruped-locomotion-rl-deploy) |
| Humanoid stand & walk (RL deploy) | [G1 / H1](#humanoid-rl-deploy) |
| G1 picks a real box, carries it, turns a corner | [Box delivery — the BATON headliner](#skill-demos--baton-sequences) |
| G1 climbs stairs *(legs-only, no vertical assist)* | [Stair climb + summit stand](#skill-demos--baton-sequences) |
| Browse / run / compose every G1 skill | [The Skill Library](#the-skill-library--one-cli-for-all-of-the-above) |
| Type to talk to a robot | [Chat demos](#1-chat-demos--single-robot-natural-language-console) |
| City traffic — follow a car | [The Living City](#7-misc--showcase) |

---

## Onboarding starter

### Warehouse Husky *(onboarding)*

The default demo. Supervisor-enabled Husky random-walks a 30 × 18 m warehouse with reactive collision recovery. Shows the URDF importer, supervisor API, motor torque pipeline, and camera follow.

| | |
|---|---|
| World | [`projects/samples/demos/worlds/showcase/warehouse_husky.omniworld`](projects/samples/demos/worlds/showcase/warehouse_husky.omniworld) |
| Controller | [`husky_random`](projects/default/controllers/husky_random/) |
| Launch | `launch.bat` *(no args)* |

---

## 1. Chat demos — single robot, natural-language console

Right-click the robot → *Show Robot Window* → a chat side panel opens. Type `home`, `wave hello`, `forward 1 m`, `turn left 90 degrees`, `stop`. Offline = regex intent router. Set `OMNI_KEY` for full LLM routing through OmniLink.

Full guide: [`docs/guide/omnilink-chat-demos.md`](docs/guide/omnilink-chat-demos.md). Index: [`projects/samples/demos/worlds/chat/OMNILINK_CHAT_DEMOS.md`](projects/samples/demos/worlds/chat/OMNILINK_CHAT_DEMOS.md).

### Arms

All four drive over the [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) HTTP surface
(§4 of [AGENTS.md](AGENTS.md)). Verified driving: joint tracking to ~0.02–0.03 rad, and damped-least-squares IK
reaching a Cartesian target to ~4.6 cm. Position control and suction / parallel-jaw picking — **no force control and
no in-hand manipulation**.

| Demo | World | Bridge controller |
|---|---|---|
| Universal Robots UR5e *(with IK)* | [`omnilink_ur5e.omniworld`](projects/samples/demos/worlds/chat/omnilink_ur5e.omniworld) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) |
| Universal Robots UR3e | [`omnilink_ur3e.omniworld`](projects/samples/demos/worlds/chat/omnilink_ur3e.omniworld) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) |
| Universal Robots UR10e | [`omnilink_ur10e.omniworld`](projects/samples/demos/worlds/chat/omnilink_ur10e.omniworld) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) |
| Three UR5e arms *(one console each)* | [`omnilink_multi_arm.omniworld`](projects/samples/demos/worlds/chat/omnilink_multi_arm.omniworld) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) ×3 (ports 8765–8767) |
| OmniArm 6 | [`omnilink_omniarm6.omniworld`](projects/samples/demos/worlds/chat/omnilink_omniarm6.omniworld) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) (`--robot omniarm6`) |

### Mobile bases

| Demo | World | Bridge controller |
|---|---|---|
| Clearpath Husky | [`omnilink_husky.omniworld`](projects/samples/demos/worlds/chat/omnilink_husky.omniworld) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| Clearpath Jackal | [`omnilink_jackal.omniworld`](projects/samples/demos/worlds/chat/omnilink_jackal.omniworld) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| Husarion Rosbot | [`omnilink_rosbot.omniworld`](projects/samples/demos/worlds/chat/omnilink_rosbot.omniworld) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| Husarion Rosbot XL | [`omnilink_rosbot_xl.omniworld`](projects/samples/demos/worlds/chat/omnilink_rosbot_xl.omniworld) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| TurtleBot3 Burger | [`omnilink_tb3_burger.omniworld`](projects/samples/demos/worlds/chat/omnilink_tb3_burger.omniworld) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| TurtleBot3 Waffle | [`omnilink_tb3_waffle.omniworld`](projects/samples/demos/worlds/chat/omnilink_tb3_waffle.omniworld) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |
| TurtleBot3 Waffle Pi | [`omnilink_tb3_waffle_pi.omniworld`](projects/samples/demos/worlds/chat/omnilink_tb3_waffle_pi.omniworld) | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) |

### Quadruped

| Demo | World | Bridge controller |
|---|---|---|
| OmniQuad *(poses only)* | [`omnilink_omniquad.omniworld`](projects/samples/demos/worlds/chat/omnilink_omniquad.omniworld) | [`omnilink_quadruped_bridge`](projects/samples/demos/controllers/omnilink_quadruped_bridge/) |

### Aerial

| Demo | World | Bridge controller |
|---|---|---|
| DJI Mavic 2 Pro | [`omnilink_mavic.omniworld`](projects/samples/demos/worlds/chat/omnilink_mavic.omniworld) | [`mavic_omnilink_bridge`](projects/samples/demos/controllers/mavic_omnilink_bridge/) |

---

## 2. Flagship demos (OmniSim engine showcases)

These showcase **OmniSim's own engine** — the URDF importer, the Newton/MuJoCo physics deploy, the RL pipeline, and the manipulation stack. **No `OMNI_KEY` / LLM needed.** (The OmniLink agent demos that used to be the flagship set now live in [§3](#3-omnilink-agent-demos-agent-layer-on-top-of-omnisim).)

**Locomotion + Humanoid (Tiers B/C)** — these are **RL deploy** worlds that need a set of Newton env vars + a policy `.onnx`, so they launch via their `scripts/dev/run_*deploy*.ps1` script (which sets that env), **not** a bare world load. Add `-Gui` to watch live.

> **Platform note.** The `bash …/*.sh` launchers run on both Windows (git-bash) and Linux; the `.ps1` launchers are Windows. As of v5.1 every deploy demo has a `.sh` sibling, so all the demos below run on Linux too (NVIDIA/CUDA GPU + the Newton wheels in the system `python3` required — see the [quickstart's Linux section](docs/developer/quickstart.md#linux-quickstart-ubuntu)).

> **Honest status is canonical.** Every one-liner below is sourced from [`docs/developer/rl-current-state.md`](docs/developer/rl-current-state.md) — the single source of truth for OmniSim RL. "Stands" ≠ "stands via RL"; "walks" ≠ "walks durably". Read it before quoting any robot result.

### Living ecosystem (alife) ⭐ *flagship artificial-life demo*

| Demo | World | Controller |
|---|---|---|
| Living ecosystem — evolved quadrupeds forage, breed and starve | [`alife_life.omniworld`](projects/alife/worlds/alife_life.omniworld) | [`terrarium_life`](projects/alife/controllers/terrarium_life/) (one supervisor drives every creature; HTTP bridge on `:8790`) |

Karl Sims-style creatures with real jointed bodies on Newton/MuJoCo. Every
creature is a `controller "<none>"` Robot actuated by one director through
batched field writes; food is visual-only; births and deaths are teleports of
pooled slots; species *bodies* evolve between epochs by regenerating the world.
Build + watch: `python projects/alife/ecosystem.py --epochs 8 --species 3
--slots 2 --alive 1 --epoch-s 90 --arena 18` then `python
projects/alife/watch_life.py`. Full measured story (what broke and why):
[`projects/alife/README.md`](projects/alife/README.md).

### OmniArm 6 pick-and-place

| Demo | World | Controller |
|---|---|---|
| OmniLink Warehouse *(OmniArm 6 + two OmniTug 500 tugs)* | [`warehouse_omnilink.omniworld`](projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld) | [`omnilink_arm_bridge`](projects/samples/demos/controllers/omnilink_arm_bridge/) (vacuum) + [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) ×2 |
| OmniArm 6 bin picking + colour sort | [`omniarm6_bin_picking.omniworld`](projects/samples/demos/worlds/flagship/omniarm6_bin_picking.omniworld) | [`omniarm6_bin_picking`](projects/samples/demos/controllers/omniarm6_bin_picking/) |

Both load and step under Newton/MuJoCo. Two caveats worth knowing before you judge
the physics: the arm's authored cylinder colliders are substituted by the solver
(the engine warns per link, naming the substitution), and the pick worlds are tuned
through their PowerShell launchers, which export a ground friction the bare world
file does not declare — a bare load is self-consistent but not launcher-identical.

### Quadruped locomotion (RL deploy)

RL-deploy walking / recovery under OmniSim Newton (MuJoCo solver). Launch each via its script; add `-Gui` to watch live.

| Demo | Launch | Status (per rl-current-state.md) |
|---|---|---|
| OmniQuad velocity-conditioned walk / stop / walk | `scripts/dev/run_omniquad_walk_vc_deploy.ps1` | ✅ Newton-verified — **47.8 m, 0 falls**, 0.32 m/s, bz 0.553 (`gpu_omniquad_walk_vc_main`); one policy walks, decelerates, stands, resumes |
| OmniQuad straight walk (G1 foot-space recipe) | `scripts/dev/run_omniquad_walk_deploy.ps1` | ✅ walks dead straight — the learned residual is **load-bearing** (+44 m with policy vs +0.13 m bare; verified 2026-06-26) |
| Go2 walk | `scripts/dev/run_go2_walk_deploy.ps1` | ✅ +86 m, 0 falls, ~0.38 m/s under Newton |
| OmniQuad crouch–recover | `scripts/dev/run_omniquad_crouch_deploy.ps1` | ✅ quasi-static; replays a certified ghost open-loop, 0 falls |
| OmniQuad get-up from the ground | `scripts/dev/run_omniquad_getup_deploy.ps1` | ✅ RL rise → faded hold, 0 falls |
| B2 get-up | `scripts/dev/run_b2_getup_deploy.ps1` | ✅ rise solved — ⚠️ the post-rise *hold* is chaotically fragile |
| B2 walk | `scripts/dev/run_b2_walk_deploy.ps1` | ⚠️ trot-model + RL-policy deploy harness present (bare-model baseline ~0.22 m/s); run it to see current forward progress |
| OmniQuad jump | `scripts/dev/run_omniquad_jump_deploy.ps1` | ⚠️ experimental deploy harness |
| OmniQuad / B2 hill walk | `scripts/dev/run_omniquad_hill_deploy.ps1`, `run_b2_hill_deploy.ps1` | ⚠️ ghost pipeline done + owner-approved — **BLOCKED** at the flat→ramp transition + slope roll-instability (~2.3 m cap) |

### Humanoid (RL deploy)

Humanoid walking + standing under OmniSim Newton. The flagship result is **OmniSim's own in-engine G1 walk, trained by Shadowing** (train == deploy bit-exact) — and it is the canonical way OmniSim makes legged-robot policies. This **supersedes** the 2026-06 "from-scratch G1 doesn't walk" finding. The **re-hosted Unitree policies** and their behavior-clones (BC) also walk durably (G1/H1). Deterministic **stands** hold for H1; the G1 needs its **active-balance (RL) stand** (`g1_hstand_deploy`), not the deterministic pose. Full canonical status: [rl-current-state.md](docs/developer/rl-current-state.md).

> ⚠️ **DISCLOSURE — THE PUPPET RIG. The flagship G1 walk is NOT a free-standing walk.** Every row and BATON sequence marked **🪝 craned** below runs on a **weight-bearing balance harness**: `HARNESS_LAM0=0.9`, `HARNESS_KZ=2000`, up to **700 N** of upward pelvis force — roughly **2× the 34 kg G1's body weight** — plus **±350 N·m** of attitude torque (the `_FCAP`/`_TCAP` clamps in [`g1_walk_recipe.py`](projects/policies/training/g1_walk_recipe.py)). The code's own comment is explicit: *"a fresh net doesn't tip over — it BUCKLES straight down… **the toddler harness holds weight, not just tilt**."* The robot is fully physical and does real footwork, but under 🪝 it is **not carrying its own weight unaided**.
>
> The **honest exception is the stair climb** (`run_climb_stairs.sh` / `run_climb_stairs_stand.sh`), which sets **`HARNESS_KZ=0`** — no vertical wire at all, so **the legs genuinely do the lifting**; the crane only trims lateral/attitude. That result stands on its own legs, literally.

| Demo | Launch | Status (per rl-current-state.md, 2026-07-04; flagship row 2026-07-06) |
|---|---|---|
| ⭐ **THE DECENT WALKER — FLAGSHIP humanoid demo (owner-designated 2026-07-06)** | [`projects/policies/worlds/run_g1_decent_walker.ps1`](projects/policies/worlds/run_g1_decent_walker.ps1) *(script only — the demo needs the full deploy env; a bare world load shows a lifeless robot)* | ✅ **owner-verified live** ("a decent looking walking robot for the first time") — **🪝 craned** (`LAM0=0.9`, `KZ=2000`; see the disclosure above): G1 walks the OFFICIAL Unitree gait on the visible-harness PUPPET rig beside its ghost hologram; LSTM+foresight champion [`projects/policies/training/runs/wr_decent_walker.pt`](projects/policies/training/runs/), **WBMATCH4 0.868** (honest shape-only ruler, exam-verified K=2048); natural thigh-clearing arm swing (ghost v3). Known-open: live pace below trainer (stride-gap thread). |
| **G1 walk (OmniSim Shadowing — canonical method)** | via [`projects/policies/training/`](projects/policies/training/README.md): `run_walk_rl.sh` deploy on [`g1_walk_ghost2.omniworld`](projects/policies/worlds/g1_walk_ghost2.omniworld) (env-driven; **VERIFY-BEFORE-SHOW** via [`verify_walkstop.py`](projects/policies/training/verify_walkstop.py)) | ⚠️ **in-engine, train==deploy — but NOT unconditionally durable, and 🪝 craned.** Durability champion `projects/policies/training/runs/wr_showpiece.pt` 45.6 m / 101 s / 0 falls; style champion `projects/policies/training/runs/wr_calm_champion.pt` WBMATCH 0.908 vs the owner-approved reference. The walk skill's own manifest ([`skills/humanoid/g1_walk/skill.json`](projects/policies/skills/humanoid/g1_walk/skill.json)) is the honest word: *"Long straights have a residual live fall rate (~open durability)."* Live pace ~0.2 m/s. |
| **G1 stair climb (walking-ghost + terrain)** | [`projects/policies/demos/run_climb_stairs.sh`](projects/policies/demos/run_climb_stairs.sh) on [`g1_climb_stairs_demo3.omniworld`](projects/policies/worlds/g1_climb_stairs_demo3.omniworld) | ✅ **full 5-step live climb @ 3 cm risers** (shipped 2026-07-08) — real foot steps, **legs-only: `HARNESS_KZ=0`**, no vertical assist (base z 0.72→0.88). ⚠️ **"stairs" are NOT solved: 3 cm is the MEASURED CEILING** for the stock-foot G1 climbing legs-only — **4 cm ≈ 2 steps, 5 cm ≈ 0** (the small-foot propulsion wall). Real staircases (~17 cm) need a bigfoot morphology or a vertical assist. Gait is also not style-clean (heading wanders up to ~124° mid-climb). |
| **G1 walk (Unitree re-host)** | `projects/policies/worlds/run_g1_unitree_walk.ps1` | ✅ **33.7 m, 0 falls, 0.48 m/s** (Unitree `motion.pt`); the team's BC clone `g1_bc_walk.pt` walks 44 m+ likewise |
| **H1 walk (Unitree re-host)** | `projects/policies/worlds/run_h1_unitree_walk.ps1` | ✅ **30 m, 0 falls, 0.42 m/s** (Unitree `motion.pt`); BC clone `h1_bc_walk.pt` walks 26 m+ likewise |
| **G1 stand (active-balance RL)** | [`projects/policies/worlds/g1_hstand_deploy.omniworld`](projects/policies/worlds/g1_hstand_deploy.omniworld) | ✅ holds via whole-body active balance where the deterministic G1 pose tips. **Deploy** figure (the `stand_rl` skill manifest, measured in the box-delivery sequence): dead-still hold — **vx 0.00, roll 0.001, z 0.778, 0 falls across all BATON switches**. *(The often-quoted "surv 1.0" is a **trainer** metric, not a deploy result — don't cite it as one.)* |
| **H1 stand** | `scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1` | ✅ holds (bz 0.977, 0 falls) — deterministic pure pose |
| H1 shadow-walk | `scripts/dev/run_humanoid_walk_deploy.ps1 -Robot h1` | ⚠️ NOT durable — feasible walking shadow tracked with stiff PD; falls ~2 s; RL tracker pending (distinct from the working Unitree H1 walk above) |
| G1 deterministic stand *(superseded)* | `projects/policies/research/runners/run_g1_stand_deploy.ps1` | ❌ deterministic pure pose tips forward, FALL@~1.38 s — use the **active-balance stand above** instead |
| Pre-Shadowing G1 from-scratch *(archived)* | [`projects/policies/research/runners/`](projects/policies/research/runners/) | ❌ historical: from-scratch face-planted ~1.3 s, ghost-built v6 ~1.44 s — **superseded by the Shadowing walk above** |

### Skill demos + BATON sequences

The **composed** demos: a walk policy hands the baton to a carry / turn / stand specialist mid-run (**BATON** = policy switching at gated handovers). Six sequences ship — four on the G1 (all 🪝 craned) and **two on the Go2, which carries no harness at all**. Each is a one-line **launcher script** — they set a large deploy env (engine pymod hook + ghost LUTs + corridors + harness + the BATON course), so **a bare world load will NOT work**; it shows a lifeless robot. Always launch via the script:

```bash
bash projects/policies/demos/run_box_delivery.sh        # [dur] [gui|headless]
```

🪝 = runs on the **weight-bearing puppet harness** — see the [disclosure above](#humanoid-rl-deploy). **Legs-only** = `HARNESS_KZ=0`, no vertical assist. **Free-standing** = no harness of any kind — the quadruped rows.

| Demo | Launch (from repo root) | Status |
|---|---|---|
| ⭐ **Box delivery — the BATON headliner** | [`bash projects/policies/demos/run_box_delivery.sh`](projects/policies/demos/run_box_delivery.sh) | ✅ **0 falls.** G1 walks to cart A, **picks a real 1.5 kg box** (physical body, proximity-gated two-phase lift — it can never levitate), carries it down the corridor to cart B, **sets it down** on real contact, walks on, takes a **real ~90° footwork corner** (TURN-LOOP, `wtz=0`), walks away, ends in a stand. 🪝 **craned**. ⚠️ The corner is only certified in the **0→90° heading band** (10/10 there; every sweep past ~95° fell or spun) — the full there-and-back shuttle needs a heading-randomized retrain. ⚠️ BATON's *"switching beats a monolith"* is an **OPEN HYPOTHESIS**, not a measured result. |
| **Box delivery — classic** *(the verified baseline)* | [`bash projects/policies/demos/run_box_delivery_classic.sh`](projects/policies/demos/run_box_delivery_classic.sh) | ✅ **verified, 0 falls** — walk → stand → pick the real 1.5 kg box → carry → place (real contact) → walk on → stand. **No corner** (crane-yaw steering). 🪝 **craned**. |
| **Walk → turn → walk** | [`bash projects/policies/demos/run_walk_turn_walk.sh`](projects/policies/demos/run_walk_turn_walk.sh) | ✅ **3/3, 0 falls** (SOLVED 2026-07-10, `72a7bb19`). Walks ~5.2 m along +x, turns a **real ~90° by footwork** (`wtz=0`, no rope — lands **90.6–95.6° actual** in 2–3 TURN-LOOP passes), then walks a **clean straight leg on the new heading**. 🪝 **craned**. |
| **Turn solo** | [`bash projects/policies/demos/run_turn_solo.sh`](projects/policies/demos/run_turn_solo.sh) | ⚠️ **experimental** (per the skill registry). The footwork-turn policy **by itself**: settle to a stand → ~90° turn on pure footwork (crane-yaw auto-OFF, `wtz=0`) → decel → hold. Most reliable from a clean stand; the in-sequence variance lives in the walk→turn handoff. 🪝 **craned**. |
| ⭐ **Stair climb + summit stand** | [`bash projects/policies/demos/run_climb_stairs_stand.sh`](projects/policies/demos/run_climb_stairs_stand.sh) | ✅ Climbs the **full 5 treads (3 cm risers)** **legs-only (`HARNESS_KZ=0`)**, then a **position-gated BATON handover** swaps in the stand specialist and the robot **holds a verified motionless stand on the top landing** (FOOT_LOG: base 0.03→2.72 m, z 0.72→0.877; final seconds — both feet past the landing line at summit height, base x-std ~1 mm; the certifier [`certify_stair_human.py`](projects/policies/training/certify_stair_human.py) gates at **< 5 mm over the final 8 s**). ⚠️ Same **3 cm ceiling** as the climb below; the climb itself is **not style-clean** (heading wanders to ~124°). |
| **Stair climb** *(5 treads @ 3 cm)* | [`bash projects/policies/demos/run_climb_stairs.sh`](projects/policies/demos/run_climb_stairs.sh) | ✅ / ⚠️ — see the [G1 stair-climb row above](#humanoid-rl-deploy). **Legs-only**; **3 cm is the measured ceiling** (4 cm ≈ 2 steps, 5 cm ≈ 0). |
| **Go2 walk → stand → walk** *(quadruped BATON, **free-standing**)* | [`bash scripts/dev/run_go2_baton_deploy.sh`](scripts/dev/run_go2_baton_deploy.sh) — or `python projects/policies/skills/skill_lib.py sequence go2_walk_stand_walk` | ✅ **verified 2026-07-13, 2 switches, 0 falls, NO HARNESS.** The first non-G1 BATON sequence: a Go2 walks under the Shadowing champion (gmatch 0.864, vx +0.418 m/s), is handed to a **deterministic hold** (`go2_stand` — `policy=None`, zero residual; a quadruped is statically stable and needs no learned stand), then handed back. Same `baton.py` that sequences the G1. Schedule `walk:12,stand:6,walk:12`. ⚠️ **The bug this demo found is the reason the support gate exists**: the naive quadruped default `always_ok` is FALSE — the first `walk:12` switch landed mid-swing (phase 0.76), the Go2 tripped and **flipped onto its back at t = 12.83 s**, then kept "walking" inverted still scoring gmatch 0.92, because a pose metric cannot see that you are upside down. The host now hands over only in the four-foot support windows a duty-0.6 trot has twice per cycle. **Never read gmatch without reading roll/bz.** Evidence venue: RunPod RTX 4090. |
| **Go2 walk → turn → walk** *(quadruped BATON, **free-standing**)* | `python projects/policies/skills/skill_lib.py sequence go2_walk_turn_walk` (env-driven; the launcher is [`run_go2_baton_deploy.sh`](scripts/dev/run_go2_baton_deploy.sh) with a caller-supplied `BATON_SCHEDULE` / `BATON_SPECIALISTS`) | ✅ **verified 2026-07-17, 3/3 headless runs, 0 falls, deterministic, NO HARNESS.** Walk +4.2 m → **~169° turn in place over 5 s with position pinned** → 10.9–13.1 m straight on the reversed heading. Schedule `walk:10,turn:5,walk:10`; the turn leg is the `go2_turn` champion (99.2 % never-fell). ⚠️ Two rules this one encodes: **score leg 2 along the ACHIEVED heading, not +x** (a clean straight leg at 169° looks like *negative* progress to a naive ruler), and **plan the schedule from the MEASURED per-leg yaw gain, not the ghost's** (the G1 turner's live gain was ~0.67). Both policies' `ONNX loaded:` lines are asserted — a missing policy silently replays the bare ghost, which scores a near-ceiling gmatch **because it is the ghost**. |
| **Box grasp — suction delivery** *(100 % physics carry)* | [`bash projects/policies/demos/run_box_grasp.sh`](projects/policies/demos/run_box_grasp.sh) | ✅ **Verified full course** — the G1 picks a **real Newton box** with a **suction-cup coupling** and delivers it with **zero kinematic writes to the box**: walk 3.8 m → hover-engage → lift → 5 m two-corner carry → press-place onto the stand → exit. Zero NaN, zero falls. ⚠️ Honest limits: the suction is a **contact-free force coupling** (the palms deliberately never touch the box — palm contact still NaNs, so a **friction grasp remains parked**), and the run is ⚠️ harnessed like every G1 demo. The kinematic-rig flagship remains `run_box_delivery.sh`. |

### The Skill Library — one CLI for all of the above

Every skill above is packaged as a **versioned manifest** binding its ghost + validator verdict + deploy env + champion checkpoint + provenance, so the same trainer and deploy stack reproduce it. **15 skills + 6 BATON sequences** behind one CLI:

```bash
python projects/policies/skills/skill_lib.py list                    # the catalogue (skills + sequences, with honest status)
python projects/policies/skills/skill_lib.py sequence box_delivery   # run a BATON demo
python projects/policies/skills/skill_lib.py preview g1_walk            # just the ghost hologram
python projects/policies/skills/skill_lib.py show g1_climb_stairs       # ghost, checkpoint, deploy env, provenance
python projects/policies/skills/skill_lib.py verify-demos            # proves the manifests reproduce the demo scripts
```

Skills span robots (G1 / H1 / Go2 / OmniQuad) and methods (Shadowing / Unitree re-host / deterministic overlay), each carrying its own `verified` / `experimental` / `open` status — `turn_in_place` is **experimental** and `climb_stairs` is **open**, and the CLI says so. Full reference: [`docs/developer/skill-library.md`](docs/developer/skill-library.md) · [`projects/policies/skills/README.md`](projects/policies/skills/README.md).

---

## 3. OmniLink agent demos (agent layer on top of OmniSim)

The full-fat **agent-driven** demos — each pairs a bespoke world with a production-grade OmniLink agent (set `OMNI_KEY` for full LLM routing; several also run offline). These used to be the flagship set; they now live here so the flagship slot can showcase the OmniSim engine itself. See [`agents/ROADMAP.md`](agents/ROADMAP.md) for the build roadmap.

### Husky Maze — vision-driven navigation

Single Husky across five mazes with progressively harder briefs. The reference OmniLink agent.

| Variant | World | What's hard | Who can solve |
|---|---|---|---|
| Simple | [`husky_maze.omniworld`](projects/samples/demos/worlds/flagship/husky_maze.omniworld) | Drive to (10, 0) | script or agent |
| Unknown | [`husky_maze_unknown.omniworld`](projects/samples/demos/worlds/flagship/husky_maze_unknown.omniworld) | Map gated; lidar wall-follow | script or agent |
| Corners | [`husky_maze_corners.omniworld`](projects/samples/demos/worlds/flagship/husky_maze_corners.omniworld) | "Visit four corners and return" | **agent only** |
| Visual | [`husky_maze_visual.omniworld`](projects/samples/demos/worlds/flagship/husky_maze_visual.omniworld) | "Find the RED cylinder via camera" | **agent only** |
| Blind | [`husky_maze_blind.omniworld`](projects/samples/demos/worlds/flagship/husky_maze_blind.omniworld) | No map, no lidar; perception tags from sidecar | **agent only** |

Agent: [`agents/production/husky_maze/`](agents/production/husky_maze/). Docs: [`OVERVIEW.md`](agents/production/husky_maze/docs/OVERVIEW.md).

### Husky Swarm — multi-robot coordination ⭐ *flagship OmniLink demo*

Four Clearpath Huskies, one coordinator, one natural-language instruction. The
agent decomposes it and fires `execute_parallel` so all four move at once —
then verifies and reports real measured positions.

| | |
|---|---|
| World | [`projects/samples/demos/worlds/flagship/omnilink_husky_swarm.omniworld`](projects/samples/demos/worlds/flagship/omnilink_husky_swarm.omniworld) |
| Bridge | [`omnilink_mobile_bridge`](projects/samples/demos/controllers/omnilink_mobile_bridge/) ×4, ports 8865–8868 |
| Agent | [`agents/production/husky_swarm/`](agents/production/husky_swarm/) — 45 tools, parallel execution, persistent waypoints/routines/memory |
| Run | `python -m omnisim run-agent --agent husky_swarm`, then [`scripts/chat_drive.py`](agents/production/husky_swarm/scripts/chat_drive.py) |
| Engine | `g1-engine` (Gemini). Other engines need their own BYOK key or return 402. |
| Status | Verified end-to-end (drives): all four drive in parallel with calibrated accuracy (−0.7%..−0.1% over 1–2 m), agent reports ground-truth positions. ⚠️ Open-loop `turn_husky` is **still broken** (~43% undershoot at 90°; an earlier ~18.7% figure did not reproduce) — the closed-loop geometry tools (`drive_to_xy`, `drive_radial`, `move_swarm_to`) are the accurate path. See the README's *Measured behaviour and known gaps*. |

### Smart House — a home run by an OmniLink agent ⭐ *the persistence showcase*

A physics-backed four-room house (thermal model, energy metering, a hinged
front door, six lights, oven / TV / coffee maker) served over the same
19-tool hub surface OmniLink's Haven smart-home agent defines — and a
benchmark that measures what a **persistent** agent is worth over an
**interactive-only** one. That axis (agents that work while you're away) is
exactly what the OmniLink paid tiers sell, so this demo is the product
argument in simulation form.

| | |
|---|---|
| World | [`projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld`](projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld) |
| Bridge (the "hub") | [`smart_house_bridge`](projects/samples/demos/controllers/smart_house_bridge/), port 8766 — hub verbs + PROTOCOL.md conformance + a benchmark-only `/scenario/*` namespace with a held sim-clock mode |
| Agent | [`agents/production/smart_house/`](agents/production/smart_house/) — 19 Haven-shaped tools; OmniLink's own `agents/haven/` (in the omnilink repo) drives the same hub unmodified |
| Run | `python -m omnisim run-agent --agent smart_house`, or the benchmark: `python agents/production/smart_house/benchmark/compare_tiers.py --mock --fake-llm` (offline) / no flags (live) |
| Engine | `g1-engine` (Gemini). Other engines need their own BYOK key or return 402. |
| Status | Verified end-to-end LIVE (2026-08-19, machine `9722d23d12a3`, real `/api/chat` turns): with the occupant present both arms are byte-identical (s1: 567 Wh, same 14 actions — persistence adds nothing when you're there, measured). While away/asleep, hourly wakes vs none: oven-left-on caught in **60 vs 480 house-min**, **5.47 vs 19.69 kWh**, kitchen peak **31.5 vs 45.9 °C** (s2); a 02:10 door breach alerted CRITICAL at the 03:00 wake, **50 vs 285 house-min** (s3); morning prep pre-warmed the house and had coffee ready before a 07:30 return vs a cold house and coffee started after walking in (s4, where the persistent arm honestly spends MORE energy — 7.95 vs 6.87 kWh — buying comfort). All numbers from `/scenario/metrics` (simulator ground truth), never the agent's self-report; results in `benchmark/results/`. The plan walls fire for real: a 5-min cadence → 402 `WAKE_CADENCE_NOT_ON_PLAN`, a 4th persistent agent → 402 `PERSISTENT_AGENT_LIMIT_REACHED` (Builder limit 3). ⚠️ Platform standing orders REGISTER but do not FIRE today (hosted tick disabled 2026-04-11) — the benchmark's wake loop is the local equivalent of the standing-order pattern, and says so. |

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
| CUDA particles smoke | [`cuda_particles_smoke_test.omniworld`](projects/samples/demos/worlds/physics/cuda_particles_smoke_test.omniworld) | CUDA particle pool resolves a regex-numbered particle pattern |
| Granular sand (rigid-body approximation) | [`granular_sand_demo.omniworld`](projects/samples/demos/worlds/physics/granular_sand_demo.omniworld) | ~300 rigid spheres piling under gravity at pebble scale — **not** the `GranularGroup` particle solver, and not cohesive |
| Newton smoke | [`newton_smoke_test.omniworld`](projects/samples/demos/worlds/physics/newton_smoke_test.omniworld) | Newton physics engine baseline |
| Newton Husky | [`newton_husky_smoke_test.omniworld`](projects/samples/demos/worlds/physics/newton_husky_smoke_test.omniworld) | Husky + Newton |
| Newton Husky swarm | [`newton_husky_swarm_drive.omniworld`](projects/samples/demos/worlds/physics/newton_husky_swarm_drive.omniworld) | Multi-Husky + Newton |
| Newton Husky head-on (×3 variants) | [`newton_husky_head_on.omniworld`](projects/robot_combat/worlds/tests/newton_husky_head_on.omniworld), [`_2`](projects/robot_combat/worlds/tests/newton_husky_head_on_2.omniworld), [`_damage`](projects/robot_combat/worlds/tests/newton_husky_head_on_damage.omniworld) | Collision + damage — see [`projects/robot_combat/`](projects/robot_combat/) for the full combat dev environment |
| BattleBox proving ground | [`battlebox_husky_proving.omniworld`](projects/robot_combat/battlebots/worlds/battlebox_husky_proving.omniworld) | Two Huskies in the new [`BattleBox`](projects/robot_combat/battlebots/protos/BattleBox.proto) arena — killsaw, pushers, side screws |
| BattleBox duel (spinner vs wedge) | [`battlebox_duel.omniworld`](projects/robot_combat/battlebots/worlds/battlebox_duel.omniworld) | Phase-2 headliner — two `BattleBot` PROTOs with different weapons + match director + broadcast cuts |
| BattleBox royal rumble | [`battlebox_royal_rumble.omniworld`](projects/robot_combat/battlebots/worlds/battlebox_royal_rumble.omniworld) | All 5 weapon archetypes free-for-all — last bot standing |
| Sponge dishwashing | [`newton_vbd_sponge_dishwash.omniworld`](projects/samples/demos/worlds/physics/newton_vbd_sponge_dishwash.omniworld) | A rigid gripper picks up a **volumetric tet-FEM `SoftBody`** and scrubs a dish with it — deformable-as-tool, on `newtonSolver "vbd"` |

### ⚠ The deformable demos do not launch from the launcher, and that is not an oversight

`newton_vbd_sponge_dishwash`, and its `Cloth` siblings
[`newton_vbd_cloth_grasp`](projects/samples/demos/worlds/physics/newton_vbd_cloth_grasp.omniworld) /
[`newton_vbd_tshirt_grasp`](projects/samples/demos/worlds/physics/newton_vbd_tshirt_grasp.omniworld),
all need **`OMNISIM_CLOTH_SELF_CONTACT=0`** to grip — measured at 24× on the tracking error
(−22.11 mm with self-contact on, −0.92 mm off). There is no `WorldInfo` or node field for it,
only the environment variable, so a world file **cannot state the configuration it works in**
and the launcher has no way to supply it. Opening one of these from the launcher gives a
degraded grip that looks like a physics bug. That is a real expressiveness gap in the format,
recorded here rather than papered over.

Run the sponge demo like this, and read the verdict out of `SPONGE_LOG` rather than the exit
code — this class of demo reports a non-zero exit *after* its controller finishes and pauses
the sim:

```bash
OMNISIM_CLOTH_SELF_CONTACT=0 \
OMNISIM_CLOTH_TELEMETRY=$PWD/.build_tmp/sponge.jsonl OMNISIM_CLOTH_TELEMETRY_EVERY=10 \
SPONGE_LOG=$PWD/.build_tmp/sponge_pads.jsonl \
python -m omnisim run-headless \
  projects/samples/demos/worlds/physics/newton_vbd_sponge_dishwash.omniworld --duration 480

# The grasp is the CORRELATION between the two logs, and neither alone proves it:
python scripts/dev/verify_deformable_grasp.py \
  --pad-log .build_tmp/sponge_pads.jsonl --telemetry .build_tmp/sponge.jsonl

# The negative control MUST leave the dish dirty:
SPONGE_CONTROL_MISS=1 ... (same line)
```

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
| The Living City | [`city_traffic.omniworld`](projects/samples/demos/worlds/showcase/city_traffic.omniworld) | Generator-driven 4×4 city — 48 cars routing with traffic signals, pedestrians on crossings, a city bus, day/night cycle. Follow a car: click it → right-click → *Follow Object* (or `F5`). Regenerate via [`gen_city_traffic.py`](scripts/dev/gen_city_traffic.py) |
| Desert ruins | [`desert_ruins.omniworld`](projects/samples/demos/worlds/environments/desert_ruins.omniworld) | Rough-terrain outdoor navigation |
| Husky rocks traverse | [`husky_rocks_traverse.omniworld`](projects/samples/demos/worlds/showcase/husky_rocks_traverse.omniworld) | Lunar/Mars-style traverse |

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
