# OmniSim RL — current state (canonical status)

> # 📍 2026-07-25 NEWTON FIXES — WHAT IS AND IS NOT INVALIDATED
>
> OmniBench's diagnosis campaign found and fixed four Newton-integration defects (gravity
> plumbing, inertia fallback, friction-cone knobs, FORCE_ODE gating — see CHANGELOG v5.3.0
> 'Fixed'). **Blast radius on previously quoted results is NARROW and enumerable:**
> `WorldInfo.gravity` was never plumbed, so ONLY worlds authoring a non-default gravity were
> wrong — in the shipped tree that is the icon-studio world, one kinematic-geometry physics
> test, and the omnibench t5/t7 scenes; every RL/deploy world uses default gravity and is
> unaffected. The inertia-fallback change bites only non-URDF dynamic Solids with no explicit
> `inertiaMatrix` (URDF robots carry their own tensors — unaffected; revert knob
> `OMNISIM_NEWTON_LEGACY_INERTIA_PRESET=1`). The friction-cone default is deliberately
> UNCHANGED (per-world opt-in knobs only) pending champion re-verification. Consequently **no
> legged-robot number in this file is invalidated by the fixes**; any future re-baseline will
> be recorded here.

> # 🗑️ ATLAS IS GONE (2026-07-17) — the robot was REMOVED from the tree
>
> **OmniLink Atlas is no longer part of OmniSim.** The model (URDF + 43 meshes), its
> `RobotSpec`, registry entry, three stand controllers, four research worlds and the
> `gpu_atlas_stand_robust` checkpoints were all deleted. Two reasons, both independently
> sufficient: the Atlas stand was a **confirmed negative result** (PPO ≈ zero-action baseline,
> ~0.8 s tip, never Newton-deployed — so nothing shipped depended on it), and the upstream
> chain of title for the v5 geometry was flagged **unclear / do-not-ship** by the licensing
> audit (internal `docs/developer/third-party-licenses.md`, blocker #9 — that doc is held from
> the public snapshot; the public licensing record is
> [`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md)).
>
> **Every Atlas row below this banner is HISTORY.** It records what was measured while the robot
> was in-tree; none of it is actionable now, and `--robot atlas` no longer resolves. The negative
> result itself is preserved in [atlas-stand-rl-journey.md](atlas-stand-rl-journey.md) — read it
> before anyone proposes re-importing Atlas. Recover the code from git history prior to the
> removal commit.

> # 📍 SATURATION HINGE + TURN SKILL + FOLDABILITY MAP (2026-07-17, three RunPod sessions ≈ $2.7) — read after the 07-12 banner below
>
> Three clean results, one new skill, one hypothesis falsified, one lever proven directional.
> All numbers are in-engine (train == deploy) on a 4090/A4500; the fold gates ran on the
> controller python (mujoco **pinned 3.8.1** — 3.10's `mj_fullM` API breaks `feasibility_certificate`).
>
> **1. `go2_turn` — a certified quadruped turn skill (the quad BATON blocker is gone).** A
> turn-in-place ghost from the analytic gait's own `wz` param PASSED `ghost_validator` + all 4
> builder gates BEFORE training (closure 14.0 mm, 33.8°/s achieved under bare PD, sat 0/750).
> The champion climbed the bar the same day: 95.5 % (zero-hook)
> → 97.5 % (+400 iters) → **99.2 % never_fell (WZ_FIXED cert arm, ≥ the 99 % bar; vx −0.003 =
> true turn-in-place)**. Champion:
> [`gpu_go2_turn_main`](../../projects/policies/research/inference/policies/gpu_go2_turn_main/)
> (`.pt` + self-contained `.onnx`). **And the live BATON `walk→turn→walk` exam PASSED the same
> evening — 3/3 headless, zero falls, ~169° turn-in-place, both `ONNX loaded:` asserted; NO
> harness (quads carry none). Skill AND sequence are `verified`**
> ([`sequences/go2_walk_turn_walk.json`](../../projects/policies/skills/sequences/go2_walk_turn_walk.json), `47bfaf20`).
>
> **2. ⛔ THE Go2 ROUND-3 LADDER WALL IS CLOSURE, NOT SATURATION — hypothesis FALSIFIED.** The new
> **`QUAD_W_SATHINGE`** hinge (penalizes PEAK torque-limit fraction, not W_TAU's mean effort — the
> intervention `9dbe57c7` designed) works exactly as built: projecting `r2t0_new` through it
> (W_SATHINGE=10, 400 iters on its own ghost) cut fold re-roll **saturation 76/750 → 35/750**
> (eval sat_frac 0.065 → **0.0145**, a ~4× crush) while holding durability (99.3 %). **And the fold
> closure did not move: p95 28.0 mm → 28.3 mm** (gate < 20). So the round-2→round-3 fold failure is
> a **foot-slip / stance-closure** problem, *not* peak saturation. The hinge is the right tool for
> the wrong robot here. Next Go2-ladder lever must target closure (contact/slip reward), not torque.
>
> **3. OmniQuad foldability — the lever is DIRECTIONAL but not yet sufficient.** OmniQuad's fast trot demands
> >limit torque, so no penalty rescues it. A slower carrier reference is the designed lever, and it
> moves the binding number monotonically: fold closure p95 **legacy 81.1 → fast-trot+W15 65.2 →
> slow-trot (vx 0.25, f 1.1)+W15 38.8 mm**, and the SUPPORT+FWP certificate now **PASSES**
> (base_frac95 0.013). Still misses the 20 mm gate and the construction re-roll still FELL
> (saturated 139/750). OmniQuad is **trending toward foldable**, not foldable. Next: even slower / lower
> body-height reference, or reduced stride.
>
> **2b (evening update). THE CLOSURE WALL HAS AN ASYMPTOTE — and the ladder stays capped at rung 2.**
> `QUAD_W_FOOTSLIP` (stance-foot horizontal-velocity penalty, gate-consistent z-threshold stance) is
> the first intervention that ever moved the fold closure: p95 **28.3 → 23.3 (W=8) → 21.7 mm (W=25)**,
> and combined with the hinge (W_FS=15+W_SAT=10) **23.7 mm** — all still above the 20 mm gate, with a
> heavy speed price (0.415 → 0.255 m/s at W=25) and the stance-slip readout barely moving (~170–200
> mm/s). The lever works and saturates; the residual ~2 mm excess is likely not reward-addressable at
> corridor 0.15 (untested next probes: fold at nb=192 — shadow-iteration.md's "under-resolving
> manufactures slip" — or a wider corridor for the roll). ⚠️ A campaign-script bug (winner-recovery
> promoted a gate-FAILED ghost on file-existence; builder writes the lut even on gate FAIL) produced
> a fake "rung 3" that was validated+trained+examined before being caught — the exam verdict
> ("slower than r2") is void as a ladder claim and the bug is fixed with a gate-pass marker. The
> validator PASSing that ghost is the documented "validator ≠ fold gates" distinction, live.
>
> **Instrument note:** a builder *crash* is not a gate *verdict* — the campaign fold phases now
> refuse to write `GATES_FAILED` unless the builder actually emitted gate lines (session 1 misread a
> `scipy`-missing ImportError as "not foldable"). The 7-controller silent-degrade warning below is
> also **resolved** (audit found 16 already hardened + 2 real defects fixed). Full session log +
> the batched campaign kit: the RunPod campaign kit (private ops tree, not in the public snapshot).

> # 📍 QUADRUPEDS ARE NOW SHADOWED (2026-07-12) — and Shadowing BEATS the residual-RL incumbent
>
> **The Go2 is the first quadruped migrated from residual-RL to SHADOWING**, the method the G1
> flagship uses. Skill: [`go2_shadow_walk`](../../projects/policies/skills/quadruped/go2_shadow_walk/skill.json).
> Ghost: the **legacy champion's own achieved gait**, recorded on the deploy-matched MJCF and
> phase-folded (64 bins) — i.e. the incumbent, certified. All binding gates PASS, including the
> one that matters: the folded lut + its declared feedforward, tracked by the **bare deploy-grade
> PD with no crane** (quads carry none), **WALKS** (0.382 m/s, closed stance contacts).
>
> **Head-to-head on the LIVE DEPLOY ruler** — 3×240 s per side, interleaved, same pod, same world,
> same physics env (only the controller line differs), both policies **asserted loaded**:
>
> | | speed | falls | \|y\| drift (max) | gmatch |
> |---|---|---|---|---|
> | **SHADOWING** (`gpu_go2_shadow_main`) | **0.429 m/s** | **0** | **0.05 m** | 0.864 |
> | legacy residual-RL (`gpu_go2_walk_main`) | 0.381 m/s | 0 | 0.26 m | — |
>
> **+12.6 % speed, 5× straighter, durability tied** (neither ever fell over ~27 min of sim each).
>
> ### ⭐ FROM SCRATCH TOO — the CORRIDOR CURRICULUM (2026-07-12, later the same day)
>
> The warm-start caveat below is now **answered, not just disclosed**. Trained **from scratch**
> (policy weights never see the incumbent), Shadowing reaches **incumbent-level deploy
> performance**: **0.370 m/s vs the incumbent's 0.381**, 231.8 m, **zero falls**, gmatch 0.832
> (live, 240 s, ONNX asserted loaded, same world). The head-to-head is therefore **no longer
> circular** — and the warm-started variant remains the *upgrade* path that beats the incumbent
> (+12.6 %).
>
> **Getting there exposed a real failure mode.** From scratch on the champion's own config
> (corridor 0.15, W_GMATCH 2.0) the policy converges to a **SHUFFLE**: it tracks the ghost
> *better* than the champion (**gmatch 0.898**) while barely moving (**0.113 m/s**). The reward
> **plateaued from it=200**, so this is a **local optimum, not an undertrained run**. The ghost
> supplies the *shape*; propulsion has to be discovered, and a tight corridor + a heavy shape
> reward let the policy satisfy the reward by marching in place. Widening it (0.30 / W_GMATCH 0.5)
> buys propulsion (0.321 m/s) and *loses* the ghost (gmatch 0.537).
>
> **The fix is a curriculum:** phase 1 **wide** (0.30 / 0.5 — discover propulsion), phase 2
> **tight** (0.15 / 2.0, warm-started from phase 1 — acquire the shape now that the gait already
> propels). In-engine: **never_fell 99.8 %**, 8.84 m, 0.369 m/s, gmatch 0.832, ydrift 0.93 m.
> Champion: `gpu_go2_shadow_curriculum/`. Recipe: `_scratch/go2_scratch/RESULTS.md`.
>
> ⚠️ **What it still does NOT establish:** the **ghost is a RECORDING of the incumbent's own
> achieved gait** (doctrine class 1a). So "from scratch" means the **policy weights**, not the
> reference. A fully de-novo pipeline needs an **MPPI-generated** ghost, and the measured MPPI
> ghost for this robot PD-replays as a **shuffle** (`build_go2_shadow_ghost.py` header). Open.
>
> ⚠️ **Read these caveats before quoting the above:**
> - **Deploy is deterministic**: the 3 runs/side are reproducibility checks, **not** independent
>   samples (shadow scored 0.429 on *every* run). Distances differ only in how much sim time the
>   wall window bought.
> - The **+12.6 % champion** is **warm-started from the legacy champion's weights**. So that
>   number is an honest comparison of two **deployed policies** and a demo that Shadowing
>   *upgrades an incumbent*. The **from-scratch** number above (0.370 m/s) is the one to quote
>   for "does the method work on its own".
> - Trade-off: shadow rides ~2 cm lower (body_z 0.272 vs 0.292) and rolls slightly more.
> - The legacy side **reproduced its own Table A record** (87.5 m / 0.380 m/s vs documented
>   86.7 m / 0.379) — the baseline was not handicapped.
>
> ⛔ **THE FIRST VERSION OF THIS EXAM WAS INVALID — and it looked GREAT.** `onnxruntime` was missing
> from the interpreter that runs *controllers* (a different python than the one the engine embeds),
> so **both** sides silently ran with **ZERO residual**, walked their bare references, and exited 0
> at `[headless] PASS`. It reported a *bigger* Shadowing win (640 m vs 411 m) — the bare ghost
> replay flatters Shadowing precisely because it **is** the ghost (gmatch 0.989 vs a 0.9998
> self-match ceiling). The tell was the **baseline under-running its own documented record**.
> Fixed in-tree: the Linux bootstrap now installs controller-side wheels into the **controller**
> interpreter and hard-gates on importing them, and the Go2 deploy controllers now treat
> "policy exists but won't load" as **FATAL** instead of degrading. ~~⚠️ 7 other deploy controllers
> still carry the silent-degrade pattern~~ **RESOLVED 2026-07-17**: an independent audit of every
> policy-loading controller (26 files) confirmed `8bc38c50` had already hardened 16; the two real
> residual defects were fixed (omniquad_rl_deploy's FATAL guarded the wrong block — the load itself was
> unguarded and a dynamic ONNX input shape false-FATALed after a successful load; and 7 controllers
> printed lying "falling back" lines before exiting 2). Zero silent-degrade load paths remain.
> Open (mid-run, not load-time): g1_stand/arms/walk_deploy catch per-tick inference errors and
> degrade silently mid-run.
> **Rule: never judge a policy run by its exit code — assert the `ONNX loaded:` line.**
>
> Not yet done: **OmniQuad/B2 Shadowing** (OmniQuad is next; B2 stiffness still unreconciled) and a **quad
> BATON sequence** (needs a `go2_turn` skill, which does not exist yet).
>
> The 2026-07-06 humanoid checkpoint below remains current **for humanoids**.

> # 📍 WHERE WE STAND (2026-07-06, maintainer checkpoint — read this first)
>
> ## ⭐ FLAGSHIP HUMANOID DEMO (maintainer-designated 2026-07-06 evening): THE DECENT WALKER
> Launch: [`projects/policies/worlds/run_g1_decent_walker.ps1`](../../projects/policies/worlds/run_g1_decent_walker.ps1).
> The G1 walks the OFFICIAL Unitree gait on the visible-harness **PUPPET** rig — an overhead crane
> that carries up to **≈2× the robot's body weight** and **holds its attitude**, at λ=0.9 (⚠️ read
> **THE HARNESS** block immediately below *before* quoting any number in this box; the legs step for
> real, the crane keeps the robot up) — beside its ghost hologram. This is the first gait in the
> lineage accepted as a visually credible walk on review of the live demo. Champion
> `runs/wr_decent_walker.pt`: **LSTM actor + REF_OBS foresight** (future ghost lookaheads in
> obs) + **HARNESS_ATT_GHOST** (the harness springs toward the ghost's recorded 2.54° sway,
> not level), **WBMATCH4 0.868** on the honest shape-only ruler (exam-verified, K=2048, DR off)
> vs the authentic official reference `ghost_official_full_v3_lut` (recentered arm swing +
> outward shoulder roll = natural, thigh-clearing arms; three maintainer-eye design catches, all
> invisible to the corridor==metric ruler — the eye audits the DESIGN, the ruler audits the
> TRACKING). Session arc 0.581 → 0.868 (commits b135a8e7, 68ced9e7). The 2026-07-17 deploy audit
> fixed the slow half-cadence launch and added a corridor-bounded, foot-generated hip-yaw heading
> trim. On machine `9722d23d12a3` (RTX 3060 Laptop, build `3ef8ce85`, Newton/mujoco_warp), the exact
> puppet world improved from **0.061 to 0.120 m/s** (final 15.04 s scored window); backward samples
> fell **23.3% to 0%**, lateral drift fell from 7.0% to 1.1% of forward distance, and max |yaw|
> fell 0.216 to 0.148 rad. This remains the same λ=0.9 weight-bearing balance harness; it is not a
> free-standing result. The WBMATCH2-era numbers below
> this box are the previous checkpoint.
>
> # ⚠️⚠️ THE HARNESS — READ THIS BEFORE QUOTING **ANY** G1 WALK / TURN / CARRY NUMBER
> **Every shipped G1 locomotion demo runs on a PUPPET rig: an invisible pelvis crane that carries
> WEIGHT and BALANCE — not just tilt.** The harness is drawn in the world (it is not hidden), and
> the trainer and the deploy apply the *same* law (so train == deploy still holds) — but a reader of
> any number below must know exactly what the crane is doing before repeating it.
>
> What it applies at the pelvis (trainer `_harness_apply`,
> [`g1_walk_recipe.py`](../../projects/policies/training/g1_walk_recipe.py) ≈L1310–1350; deploy
> mirror ≈L4583–4675 — identical law):
> - **Vertical bungee** — `HARNESS_KZ=2000` N/m toward `HARNESS_Z0`, damped `HARNESS_DZ=150`,
>   **upward-only**, clamped at **700 N**. The G1 weighs 34.1 kg ≈ **335 N**, so the cap is
>   **≈2× body weight**: at the cap the robot is being *carried*. The code says it plainly —
>   *"The toddler harness holds weight, not just tilt"* (L1324), *"the harness carries balance"* (L2100).
> - **Attitude spring** — roll/pitch PD (`HARNESS_KP=600`, `HARNESS_KD=60`), clamped **±350 N·m**.
> - **Lateral catch** `HARNESS_FY=400` (in the target-heading frame) + **yaw steer** `HARNESS_KYAW=150`.
> - Every term scaled by **λ = `HARNESS_LAM0` = 0.9** — i.e. **90 % of full crane authority**. The
>   trainer *anneals* λ down as a policy graduates (`HARNESS_STEP` 0.1 per rung once eval survival
>   > 0.9), but **every shipped demo still runs at λ=0.9, and no G1 walk champion has been shown at
>   λ=0.** The crane cuts out once the robot is *already* toppling (|roll| or |pitch| > 0.6 rad, or
>   base z < 0.35 m) — so it is a balance **assist**, not a fall-catcher.
>
> **Which demos ride it:** the flagship decent walker
> ([`run_g1_decent_walker.ps1`](../../projects/policies/worlds/run_g1_decent_walker.ps1) L38–39) and —
> through the shared profile
> [`skills/profiles/g1_shadow_deploy.json`](../../projects/policies/skills/profiles/g1_shadow_deploy.json) —
> **box_delivery, walk_turn_walk and turn_solo**. Same rig, same λ=0.9, same `HARNESS_KZ=2000`.
>
> **The one exception — and it is to the project's credit:** the **stair climb**
> ([`run_climb_stairs.sh`](../../projects/policies/demos/run_climb_stairs.sh)) sets **`HARNESS_KZ=0`**:
> **no vertical assist at all — the legs do 100 % of the lifting.** (Its lateral catch and attitude
> spring are still on at λ=0.9, so the honest phrasing is "legs-only *vertically*", not "unassisted".)
>
> **The honest one-line statement of the flagship is therefore:** *the G1 walks the official Unitree
> gait with real stepping and real contact physics, while an overhead crane carries up to ≈2× its
> weight and holds its attitude.* **Removing the harness (annealing λ → 0) is an OPEN campaign, not
> a shipped result.** Never quote a G1 walk/turn/carry number without stating λ=0.9.
>
> **THE GHOST DESIGN LOOP is complete and validated** — five maintainer design corrections, five
> measured ceiling raises: **0.784 → 0.905 → 0.906 → 0.918 → 0.934** (WBMATCH2, clean-physics
> self-exams). The loop: **idealize a target → [`ghost_doctor.py`](../../projects/policies/training/ghost_doctor.py)
> gates + prescribes → burst-prove (reward-side refs, control corridor NEVER moves) →
> [`REC_FOLD`] fold-record what the robot actually does → [`ghost_polish.py`](../../projects/policies/training/ghost_polish.py)
> (symmetrize + smooth + clip) → maintainer previews the hologram → repeat.** Idealized references
> never train raw; the robot earns them into existence. Full method:
> [ghost-design-rules.md](ghost-design-rules.md) (incl. the 2026-07-05/06 laws).
>
> **PACKAGED AS SKILLS.** Shadowing + BATON are wrapped into a first-class **skill library**
> ([`projects/policies/skills/`](../../projects/policies/skills/) · [skill-library.md](skill-library.md)):
> each skill (walk / turn / carry / stand / climb, + H1 / Go2 / OmniQuad) is ONE versioned manifest
> binding its ghost + `ghost_validator` verdict + deploy env + champion checkpoint + provenance, and
> `python skill_lib.py sequence <name>` composes them into a BATON demo (box_delivery, walk_turn_walk).
> `skill_lib.py verify-demos` proves the manifests reproduce the hand-written demo scripts
> **key-for-key on the assembled launch env** (it parses each script into a KEY=VALUE env dict and
> diffs it against the manifest-assembled bundle — *not* a byte comparison of the scripts).
> **To make a new skill or build a BATON demo, start there** — it is the standard pipeline
> (design → validate → preview → train → verify → register → sequence).
>
> **Champions (all committed, all verified live in the wide ghost arena — all on the λ=0.9 harness
> rig; the live-distance records below are craned walks, see THE HARNESS above):**
> - `runs/wr_v13_it250.pt` — **THE champion**: humanized 19° stride, STRAIGHT parallel arms
>   (elbow 1.70 = extended; G1 convention: 0 is the 90° bent carry!) swinging functionally,
>   rock at the MEASURED minimum (6.1° — the sway↔stance physics floor), **0.933 @ surv 1.000**
>   vs ghost v14; live records: 26.2 m verify / 32.4 m demo lap, zero falls.
> - `runs/wr_v11_it200.pt` — functional-swing champion (0.934 @ 1.000 vs ghost v12).
> ⚠️ **2026-08-22 — the artifacts named here are NOT in the public snapshot.** LAFAN1 was
> verified as CC BY-NC-ND 4.0: the ND term withholds the right to share derivatives at all,
> and NC is incompatible with Apache-2.0. The HOP-1 ghost lineage and everything trained
> against it are release-denied. This history is left exactly as written — see
> `docs/developer/motion-data-provenance.md`.
>
> - Ghost lineage in [`ghosts/g1/`](../../projects/policies/ghosts/g1/):
>   v12 (functional swing), **v14 (calm, THE reference)** — every one fold-recorded,
>   doctor-PASS, achieved-provenance.
> - Legacy champions (wr_showpiece 45.6 m durability, wr_calm_champion 0.908 style,
>   wr_vc9_it100 walk↔stand) remain for their niches.
>
> **THE demo** (both luts to the SAME file family, `G1_GHOST_LOCK=clock`):
> world `g1_walk_ghost_wide.omniworld`, `GHOST_LUT_JSON=ghost_hop1v11_lut.json` (control),
> `G1_GHOST_LUT=ghost_v14_lut.json` (hologram), corridors `ELBOW_TARGET=1.6 ELBOW_RESIDUAL=0.10
> SHRY_TARGET=0 SHRY_RESIDUAL=0.10 ARM_RESIDUAL=0.10 ARM_SWING_A=0.20`, policy wr_v13_it250.
> ⚠ `G1_GHOST_LUT` (hologram) ≠ `GHOST_LUT_JSON` (corridor) — set both.
>
> **VERIFY-BEFORE-SHOW (maintainer rule, absolute):** no GUI until the automated verdict passes
> ([`verify_walkstop.py`](../../projects/policies/training/verify_walkstop.py) or a telemetry
> check). **MEASURE-BEFORE-CLAIM:** shape claims come from the single-cycle/folded RECORDING,
> never from scores or readouts (three false breakthroughs were caught this way); mimicry exams
> run CLEAN physics (`MOTOR_RAND=0 OBS_NOISE=0` — DR leaks into evals by default).
>
> **Measured laws (per-robot, calibrated — the doctor enforces them):**
> - Corridor center + learned offsets are INSEPARABLE: never move the leg-corridor center
>   (5 failures incl. rebase-onto-own-gait = surv 0.057); reward/metric refs are the safe knob;
>   hop-and-settle (move a proven-safe step, FREEZE, re-adapt) is the amplitude mechanism.
> - Channels resist reward (3-for-3: arms, amplitude, symmetry) → corridors; attitude is the
>   reward-responsive exception — down to its physics floor only (6.1° rock at this stance).
> - amplitude↔cadence↔speed must be jointly consistent (k measured from the robot's own
>   recordings); single-cycle recordings preserve TREMOR as reference (always fold);
>   G1 elbow 0 = bent carry, straight = +1.6.
>
> **Open fronts (in priority order):**
> 1. **Smoothness burst** (running): anti-chatter penalties vs the new SMOOTHNESS eval readout.
> 2. **Honest 0.95**: engine-recorded link positions (kills the 4 cm hand-origin floor in the
>    links term) + optionally distribution-scored attitude.
> 3. **Below 6° rock** (structural): wider stance hop-campaign or faster cadence.
> 4. **The speed frontier** (0.7+ m/s): unlocks true human striding (hip 38°+) — biggest visual
>    payoff, biggest campaign.
> 5. **Walk-stop-walk brake skill** (parked with tools: `stopv`, `W_STOP`, edge-state bank) and
>    **the dances** — both inherit the complete design loop.
>
> # ⭐ CANONICAL (2026-07-03, maintainer directive): **SHADOWING is THE algorithm for legged-robot
> # motion policies in OmniSim** — promoted to flagship at [`projects/policies/training/`](../../projects/policies/training/README.md).
>
> **Shadowing** = train in-engine (train == deploy bit-exact; the 3 deploy gaps of 2026-07-02 are
> solved) to shadow a **ghost** (an achievable, recorded reference — see the
> [ghost design rules](ghost-design-rules.md) and the calibrated pre-training
> `ghost_validator.py`) via **corridors** (style structural, balance learned), scored by
> **WBMATCH**, gait changes via **GHOST-MORPH** (never snap). Validated end-to-end on the G1:
> live-verified durable walking, WBMATCH 0.913 vs the maintainer-approved reference (champion
> `projects/policies/training/runs/wr_calm_champion.pt`).
>
> **SEQUENCE (dance/mocap) CAMPAIGN — PARKED 2026-07-04 (maintainer call), lessons banked:**
> 1. **Kinematic retargeting transfers poses, not weight placement.** Every kinematic gate can
>    pass (salsa: 96% skeleton match) while the robot's COM sits outside its own support polygon
>    67% of beats. Run **GATE 1d** (`ghost_balance_gate.py`, robot mass model vs its own foot
>    rects) on ANY sequence ghost BEFORE training — its per-beat margins rank-matched per-segment
>    training survival exactly, and it retro-explains the Charleston plateau.
> 2. **Diagnosis ladder that worked:** feasibility map (`SEQ_EVAL_MAP=1`) → segment isolation
>    (`SEQ_BIN_LO/HI`: train one slice alone; separates "segment infeasible" from "transitions
>    break") → balance gate. Tempo-slowing a dance made it WORSE (longer narrow-stance holds).
> 3. **Fix stages now in the retargeter** (all spec-driven, any robot): `--stance-min` (support
>    geometry), `--balance-min` (LIPM/DCM root re-plan — real but partial; residual = limb
>    momentum), `--audit`, and **`ghost_screen.py`** (rank candidate motions by dynamic
>    trainability — the screen independently ranked our problem salsa last, 29/66 pts).
> 4. **The construction that WORKED end-to-end:** screen → soft-style corridors → **EVAL_RECORD
>    re-record** → achieved ghost with **0% balance violations** (a real G1 danced every bin:
>    charleston 93_04, surv 0.71, 43.8% full-routine completions — `ghost_dance9304_achieved_lut`).
> 5. **NEXT (the right-sized test, maintainer direction): walk-stop-walk** — a 5s-walk/5s-stand/5s-walk
>    sequence ghost composed from the ALREADY-achieved walking reference (v3c) + the stand:
>    isolates the TIME AXIS (transitions, chaining) with zero new dynamics content. Dances resume
>    after that axis is owned.
>
> # ⭐ BATON — the sibling policy-switching paper (field-positioning settled 2026-07-08)
> BATON keeps independently-trained specialist policies (walk/stand/carry/turn) and engineers the
> runtime **handover** between them. Full paper scaffold: [policy-switching.md](policy-switching.md)
> (read its **"Where BATON stands vs the field"** canonical section before claiming any BATON edge).
> The head-to-head competitor (opposite approach = one unified policy) is LHM-Humanoid — the
> single-policy curve to beat: 90.8→76.1→61.0→38.5→20.9% over 5 cycles; it admits longer horizons
> are unstudied.
> **Verdict, so no one over-claims:** the field is moving to ONE policy for everything (universal
> trackers GMT/HOVER/ExBody2, foundation models BFM-Zero/Task Tokens — small nets, real-time on a
> Jetson, often beating specialists). "One policy can't scale to a long composed job" is **UNPROVEN**
> (LHM-Humanoid does walk→pick→carry→place on a single policy and beat a hierarchical baseline
> 72/21%) — but so is "the monolith wins": LHM's baseline used a **naive oracle-FSM handoff**, its
> own data shows a single policy degrades over horizon (90→18% across 5 cycles), and it is sim-only.
> **BATON's real claim = a well-posed OPEN GAP:** does an *engineered* handover (morph + phase-gate +
> **recurrent hidden-state management** — the STAND-ATTRACTOR LOCK / cold-zero fix, novel: all prior
> switching work is feedforward) degrade more gracefully over long horizons than a single policy?
> **Proof pending an experiment:** BATON vs single-distilled-policy vs naive-FSM-hierarchy on a
> many-cycle task, plotting success-vs-horizon (via `baton_metrics.py`). Until that plot exists the
> edge is a HYPOTHESIS, not a result.
>
> # ✅ SHIPPED DEMOS — canonical adjudications (added 2026-07-11)
> These four results ship publicly and had **no entry in this file** — a breach of this doc's own
> drift rule. Here is the honest adjudication of each. (All four are verified against the demo
> scripts / skill manifests / commits cited.)
>
> ### 1 · G1 STAIR CLIMB — real, at **3 cm risers**, legs-only vertical. **3 cm is the CEILING.**
> [`run_climb_stairs.sh`](../../projects/policies/demos/run_climb_stairs.sh) (policy `wr_stairwalk3d`,
> world `g1_climb_stairs_demo3.omniworld`): the G1 walks in on the flat and **climbs a 5-tread staircase
> with real foot steps** — each foot lifts and lands ON the next tread, base z rises 0.72 → 0.88
> (live FOOT_LOG-verified). **`HARNESS_KZ=0` — no vertical crane assist: the legs do the lifting**
> (lateral catch + attitude spring remain at λ=0.9, so: legs-only *vertically*).
> - ⛔ **3 cm is the MEASURED CEILING, not a chosen setting.** Per the demo script's own header:
>   at **4 cm the stock-foot G1 gets ≈2 steps, at 5 cm ≈0**. That is the small foot's **propulsion
>   wall**; a taller staircase needs a bigfoot morphology or a vertical assist. This demo ships what
>   the STOCK G1 genuinely can do — say "3 cm" whenever the climb is mentioned.
> - **The method that cracked it:** NOT a bespoke climb ghost — 7 climb-ghost approaches all failed
>   (feet shuffled at the base). Warm-start the EXISTING walker and drive it with the **WALKING**
>   ghost on **stair terrain** under a riser curriculum; the terrain forces it up.
> - ⚠️ **The 7 cm climb-ghost is a separate and still-OPEN result — and it is this repo's cleanest
>   proof that a validated ghost is NECESSARY, NOT SUFFICIENT.** The composed walk→climb ghost for
>   the original 7 cm × 26 cm staircase **PASSES the ghost validator and all the ghost gates
>   (`validator: PASS`, provenance `solved`)** — and **no policy climbs it live.** In the trainer it
>   **plateaus at ~2 of 5 steps** across every lever tried (propulsion — not balance, timing, or
>   height); in live deploy the policy **refuses the first riser**. Manifest verdict
>   ([`skills/humanoid/g1_climb_stairs/skill.json`](../../projects/policies/skills/humanoid/g1_climb_stairs/skill.json)):
>   status `experimental`, verification **"OPEN … not demo-ready."** Do not present the 7 cm
>   staircase as climbed.
>
> ### 2 · BATON BOX DELIVERY — 3/3, **0 falls** — **on the PUPPET rig**
> [`run_box_delivery.sh`](../../projects/policies/demos/run_box_delivery.sh) ·
> [`sequences/box_delivery.json`](../../projects/policies/skills/sequences/box_delivery.json): the G1
> walks to cart A, takes a real 1.5 kg ODE box (⚠ 2026-08-08: no longer an ODE body — ODE was
> deleted in `bdc02139` and Newton is the only backend; the box's own physics config was not
> re-verified) with a proximity-gated two-phase lift (it can never
> levitate the box across the room), carries it 4.6 m down the corridor, **sets it down** at cart B's
> rest point (9.00, −1.40, 0.960) with a real contact settle, walks on, takes a real ~90° footwork
> corner, walks away, ends standing. **Re-verified 2026-07-17 on machine `9722d23d12a3`
> (RTX 3060 Laptop, build `f0e395c3`): 11/11 segments in 91.8 simulated seconds, pickup at
> 19.4 s, payload cruise ~0.28 m/s (previously ~0.19), exact cart-B settle
> `(9.00, -1.40, 0.960)`, 89.3° footwork corner, min base-z 0.658, 0 falls.**
> - ⚠️ **Caveat 1 — the rig.** This is the `g1_shadow_deploy` **PUPPET** (λ=0.9, `HARNESS_KZ=2000`).
>   The pick, the carry, the place and the corner are real; **the balance is craned.** See THE HARNESS.
> - ⚠️ **Caveat 2 — the heading band.** The turner's ghost spans **exactly 0→90° of ABSOLUTE
>   heading**, and every corner whose sweep crossed ~95° fell or spun (measured at 103 / 106 / 113 /
>   ~150°). The there-and-back shuttle is therefore **NOT shipped** — it needs a heading-randomized
>   retrain of the turner (the named next step).
> - ⚠️ **Caveat 3 — BATON's headline claim is an OPEN HYPOTHESIS.** "An *engineered* handover
>   degrades more gracefully over a long horizon than a single policy" is the paper's claim, and
>   **the success-vs-horizon experiment has not been run** (see the BATON section above). **Zero
>   falls on one ~170 s course is not that curve** — do not let the demo stand in for the result.
>
> ### 3 · IN-SEQUENCE 90° FOOTWORK TURN — **SOLVED** (`72a7bb19`)
> [`run_walk_turn_walk.sh`](../../projects/policies/demos/run_walk_turn_walk.sh): the G1 walks ~5 m,
> turns a **real ~90° by FOOTWORK** mid-sequence (the crane's yaw steer auto-disables during the turn,
> wtz=0 — the legs do the rotating), then walks a clean straight leg along the NEW heading.
> **Measured 3/3: 90.6–95.6° ACTUAL heading, 0 falls.**
> - **Mechanism = the TURN-LOOP.** The 90° step-turn ghost played once banks only **60–65 %** of its
>   yaw live (foot slip; measured gain **0.67**, stable across runs). The ghost is a *modular
>   staircase* of 15° feet-together mini-pivots, so the deploy **replays partial passes** — restarting
>   at the plateau whose remaining staircase ≈ remaining angle ÷ measured gain — until the **actual
>   accumulated heading** reaches target. Landing quantum = the 15° mini-pivot.
> - ⛔ **Never stop a sequence ghost mid-lut**: both measured mid-lut arrests recoiled −19° or spun
>   and fell. Every pass must play through the ghost's own decel into its end-hold.
> - Same puppet rig (λ=0.9). Within the heading band of §2.
>
> ### 4 · IN-ENGINE QUAD TRAINER — Go2 / OmniQuad / B2, train == deploy (`a824e564`, `101864a4`)
> [`quad_walk_recipe.py`](../../projects/policies/training/quad_walk_recipe.py) +
> [`run_quad_walk_rl.sh`](../../projects/policies/training/run_quad_walk_rl.sh) port the quads onto the
> G1's in-engine path: the K batched rollout worlds come from `world._mpc_rollout_buffers(K)` — **the
> SAME compiled MjModel the live deploy `SolverMuJoCo` steps** — closing the quads' last train→deploy
> parity gap (the standalone trainers' MJCF re-parse). One recipe, three quads (`QUAD_ROBOT=go2|omniquad|b2`),
> each warm-started from its flat champion. It also fixed a **dishonest eval**: `done` used to include
> the 12 s episode timeout, so `never_fell` read 0 % for a policy that never fell.
> - **Measured — a TRAINER-side batched eval on the deploy-identical model, NOT a live single-robot
>   deploy run:** **Go2** 400 iters / 4096 envs, 19.7 M steps @ ~77 k env-steps/s → **94.8 % never
>   fell over 48 s, 16.6 m mean (max 20.7 m), 0.357 m·s⁻¹**. **OmniQuad** (30-iter *smoke*, 1024 envs) →
>   **69.5 % over 24 s, 6.0 m, 0.304 m·s⁻¹**. **Do not quote 94.8 % as a deploy result** — the live
>   Newton deploy walks on record for the quads are still the *standalone*-trained ones (Go2 +86.7 m,
>   OmniQuad +47.8 m, B2 +110.7 m, 0 falls — Table A in the historical block); the in-engine champions
>   have **not** yet been given an equivalent live long-run.
> - ⚠️ **B2 stiffness is UNRECONCILED.** `b2_walk_deploy.py` documents KE=80 / KD=2.0 — which looks
>   copy-pasted from the Go2 and is implausible for a ~60 kg quad — while `run_quad_rough_track.ps1`
>   uses 1400 / 35 (the launcher's default). **Reconcile before trusting any B2 number.**
> - **Venue:** local, in-engine. The `cloud/` Modal wrappers were **REMOVED** (`ef46a52e`); there is
>   no cloud training path in OmniSim and none should be added.
>
> **Supersessions, to keep this file contradiction-free:**
> - The **⏹ deterministic-first banner (2026-06-27)** below: still true for *static stand/balance*
>   (the deterministic stand remains the launch/settle layer the Shadowing pipeline itself uses),
>   **superseded for locomotion and motion** — Shadowing is the shipped path there.
> - The **"Shadowing is the wrong architecture for the H1 walk" course-correction (2026-06-25)**
>   in the robot table below: that verdict judged the PRE-PARITY era (standalone trainer, the
>   dt/handoff/clamp deploy gaps unsolved, hand-designed kinematic ghosts). Post-parity, in-engine
>   Shadowing with a RECORDED ghost is exactly what produced the durable G1 walk. The H1 entry's
>   *data* stands as history; its *conclusion* does not.
> - Older per-robot "walls" (G1 forward-tip, H1 ~2 s) were artifacts of the since-solved
>   train↔deploy gaps + infeasible references, per [train-deploy-gap.md](train-deploy-gap.md) and
>   [g1-walk-recipe.md](g1-walk-recipe.md).

> # ✅ UPDATE (2026-07-01): a durable from-scratch G1 WALK is now SOLVED in-engine.
>
> **The "never reached / ~1.4 s forward-tip wall" verdict below is SUPERSEDED for the walk.**
> A G1 now walks forward, upright, by real stepping, trained in OmniSim's own mujoco_warp
> engine (train == deploy, zero sim gap): **~8 m at surv=0.92 over a 3000-step horizon,
> gmatch=0.87** (genuine stepping, not diving); live GUI deploy stayed upright ~1300 steps /
> ~15 m of ground path. The path was **the field's deploy-proven RECIPE** (from-scratch
> velocity-command tracking + asymmetric **privileged critic** + domain randomization +
> position-target PD, per a 10-source sweep of Unitree/Playground/Booster/Berkeley/etc.) **plus
> four diagnosed fixes**: a soft **GHOST leg-imitation reward** (kills the dive-and-faceplant
> reward-hack → durable stepping), a **tent (V-shaped) velocity reward** (forward without
> lunging), and **heading-in-obs** (walks straight, not in a curve). Full detail + how-to-run:
> **[g1-walk-recipe.md](g1-walk-recipe.md)**. Honest open item: a residual heading drift caps
> it (~15 m path then a topple) — not yet a dead-straight infinite walk. Note also that the
> earlier "residual-RL-on-the-deterministic-gait walks" claim was **wrong and retracted** (it
> falls ~3 s / 1.15 m). The deterministic-first direction below still holds for STAND/balance;
> **RL is the working path for LOCOMOTION** — quadruped, and now the from-scratch humanoid walk.
>
> ---
>
> # ⏹ STRATEGIC DIRECTION (2026-06-27): OmniSim is now DETERMINISTIC-CONTROL-FIRST
>
> **Reinforcement learning is treated as an OPEN, UNSOLVED problem — not a shipped path.**
> The program's headline RL goal — a **durable, deployable from-scratch humanoid** (G1/H1)
> walk or stand — was **never reached**: every OmniSim-trained humanoid policy hits a
> **~1.4 s forward-tip wall** in the real Newton deploy (the detail below documents this in
> full). After months on it, the verdict is that this RL effort **did not get us a useful
> deployable humanoid policy.**
>
> **What works instead is deterministic, model-based control.** The proof point (2026-06-27):
> a G1 that **stands and robustly absorbs cubes thrown from every side**, returns to upright
> after each hit, and shifts its weight onto one leg — all from a **deterministic** stiff-hold
> + capture-point + arm/hip-balance controller with **no learned policy** (`projects/policies/
> controllers/humanoid_stand_deploy/`). It survives harder perturbations than any RL attempt,
> and a measured A/B test showed that **adding learned/active feedback made it WORSE**. From
> here, OmniSim builds forward on deterministic control; the RL pipeline (now under
> `projects/policies/`, formerly `projects/rl/`) is retained only as a research track.
>
> **Honest scope (this doc is the single source of truth and must stay truthful):** two RL
> results below ARE real and verified — OmniSim-trained **quadruped** walks (OmniQuad/Go2/B2,
> 0 falls) and **re-hosted Unitree** humanoid policies. Those are not failures. But they are
> **not** the from-scratch humanoid capability the program was chasing, and the strategic
> direction is deterministic-first regardless. Everything below is the (now de-prioritised)
> RL status record, kept for the honest history and for the research track.
>
> **Quadruped LOCOMOTION stays on RL — measured (2026-06-27).** A direct attempt to replace
> the RL quad walk with deterministic control was run and measured on real Newton (OmniQuad,
> `omniquad_walk_deploy.omniworld`): RL residual = **0.40 m/s, dead straight, 0 falls, 327 s+ (131 m)**;
> the bare deterministic trot base (`-Bare`) = **0.036 m/s and drifts backward** — it barely
> propels, it only provides standing stability; the velocity-feedback Raibert walker
> (`omniquad_raibert_walk`) propels (0.16–0.31 m/s) but has an unsolved **roll instability** that
> flips it in 3–31 s, and raising attitude/heading gains made it *worse*. The bare base goes
> **−x** while the residual-on policy goes **+x** on the same gait, so the RL "residual"
> (±0.15 rad) is doing **nearly all** the propulsion, not a small correction. Deterministic
> quad locomotion at RL parity is therefore a real convex-MPC / attitude-control project, not
> a config swap. Decision: **RL is retained as the working approach for quadruped walking**;
> deterministic owns the quasi-static stand/balance/cube-defense. Don't re-run the bare-trot /
> Raibert sweep expecting parity — it's been measured.
>
> **Terrain-curriculum RL extends the quad walk to ROUGH ground — VERIFIED (2026-06-28).** The
> flat-trained residual walk is blind to terrain (limit ≈ gait foot-clearance: OmniQuad trips at a
> 7 cm bump, B2 at 14 cm). Putting randomized bump bars into the training MJCF (blind
> `--rough-amp` mode, ground-relative reward, obs unchanged so the deploy controller is
> untouched) fixes it: terrain-trained **OmniQuad clears 18 cm bumps + a rubble field, 0 falls,
> 70 m+**; **B2 clears 14 cm** (up from 10). Trained via the standalone terrain trainers
> (`gpu_mjwarp_*_walk_trainer.py --rough-amp`, run locally on the GPU). Eval:
> `run_quad_rough_track.ps1 -Robot <r> -Policy <onnx> -ActScale 0.25`. Two caveats: the
> curriculum must be **capped per-robot** (a too-tall top stage over-trains and the policy
> forgets to stand on flat — pick the best stage), and **heading-on-terrain** (steering vs
> robustness) is an open tradeoff (the terrain policies drift; the `--wz-range` fix competes
> with stability). This is RL (residual on the trot), consistent with the decision above.
>
> ---

> ## 🎉 2026-06-27 — CLOSED-LOOP train↔deploy gap CLOSED for the G1 STAND (this section supersedes the G1-stand rows below)
>
> **Milestone — the day we identified and fixed the train↔deploy gap.** A from-scratch
> (non-Unitree, non-BC) **G1 RL stand now survives in the real `omnisim-bin` deploy:
> 32.0 s / 1998 ticks, 0 falls** (headless AND GUI; `OMNISIM_REQUIRE_NEWTON=1`, Newton
> `mujoco_warp`; min base-z 0.696, flat). This is the **first OmniSim-trained humanoid
> policy to stand durably in the binary**, and the first time the closed-loop train↔deploy
> gap has been **pinned to a specific bug and closed by fixing it** rather than worked around.
>
> ### What the gap actually was (it was the OBSERVATION pipeline, not the physics)
> Open-loop physics parity was already proven (welded binary probe ~1e-5 rad, below). The
> remaining deploy fall was a **systematic observation-pipeline bug**, found by a new
> closed-loop parity test
> ([`projects/policies/research/training/closed_loop_parity_compare.py`](../../projects/policies/research/training/closed_loop_parity_compare.py))
> that runs the SAME policy through the certified `add_urdf`+SolverMuJoCo path AND the real
> binary while **logging the observation vector each side fed the policy** (not just the
> resulting state), then diffs per component — reading the *early* ticks (before chaos can
> grow) to separate a systematic bug from chaos.
> - **The bug:** the base **angular-velocity** obs term. The trainer reads mujoco free-joint
>   `qvel[3:6]` (body frame); the deploy controller fed `R^T·getVelocity()[3:6]` from the
>   OmniSim Newton backend — a **different frame/scale** (norms 2.13 vs 1.67, yaw/z
>   **sign-flipped**). At tick 1 it diverged by **~2 rad/s while the pose still matched to
>   3e-3** → the policy received an out-of-distribution signal → toppled at tick 111 (~1.8 s).
>   Proof it was the obs and not physics: the certified eval (reads `qvel` = exactly what the
>   policy trained on) **STOOD**; the binary (fed the different signal) **FELL**.
> - **The fix:** drop the engine base ang-vel from the obs (it was always the "hardest to
>   reproduce in deploy" term the obs design itself warned about) and replace it with the
>   **finite-difference of `proj_gravity`** — reproducible to ~3e-3 on both sides, carries the
>   balance-relevant roll/pitch rates, drops the unreproducible yaw. Applied **identically** in
>   the trainer obs, the certified eval, and the deploy controller. Obs stays 32-dim (no net
>   network change).
>
> ### Measured result (commit `1416d52c`)
> | metric | before (engine ang-vel) | after (reproducible d·proj_gravity) |
> |--|--|--|
> | tick-0 closed-loop obs divergence | 0.129 | **0.004** (30×) |
> | binary deploy (`g1_newton_stand_dr2`) | **fell ~1.8 s** | **STANDS 32.0 s, 0 falls** |
> | trainer survival (64 env × 400 step) | 398/400 | 398/400 |
>
> Policy: `projects/policies/research/training/runs/g1_newton_stand_dr2/policy.onnx`. Reproduce:
> `scripts/dev/run_g1_policy_probe.ps1 -Gui -Onnx projects/policies/research/training/runs/g1_newton_stand_dr2/policy.onnx -Settle 40 -Ticks 2000`.
>
> ### Scope — read this before over-claiming (the honest boundary)
> - **Supersedes** Table C's "G1 stand REFUTED" **only for the RL stand**. The *deterministic
>   pure-pose* G1 stand still tips ~1.38 s — unchanged; that is a **different artifact** (a
>   pose hold), not this trained 32-dim policy on certified physics. Do not conflate them.
> - **Does NOT** touch the from-scratch G1 **walk** (Table B): those policies were not retrained
>   with this obs fix and remain at the ~1.4 s wall. But they feed the **same** engine base
>   ang-vel, so the **same bug very likely contributes** — re-running
>   `closed_loop_parity_compare.py` on a walk policy is the next lever.
> - **Generalizes:** any deploy obs term sourced from the OmniSim Newton `getVelocity()` angular
>   part is suspect; derive rates from a **reproducible orientation finite-diff** instead.
>
> ### Standard diagnostic that came out of this — use it on EVERY train↔deploy puzzle
> The same closed-loop tool is now a hardened **BUG-vs-CHAOS classifier**
> ([closed-loop-chaos-diagnostic.md](closed-loop-chaos-diagnostic.md)). Run the same policy on
> both sides and it reads the *shape* of the divergence: diverge-from-tick-1 = a real **`[BUG]`**
> (it even catches an obs bug hiding under chaos and names the channel); matched-for-~0.3 s-then-
> exponential = **`[CHAOS]`** (intrinsic instability, NOT a pipeline bug — fix with robustness, not
> parity); matched-throughout (welded) = **`[MATCH]`**. **Before claiming "the physics is wrong" or
> "the gap is open," run this.** A free biped is chaotic (positive Lyapunov exponent): the G1 stand's
> trainer and binary trajectories match to 5 mm for 0.3 s then e-fold every 0.27 s — that 4 m "slide"
> is chaos + the policy's walk, not a bug. Chaos can never be zero for a free base; the welded lane
> matches to ~1e-5 precisely because welding removes the instability.
>
> ---

> ## ✅ EMPIRICAL RE-VERIFICATION — 2026-06-26 (this section supersedes everything below it)
>
> Every number in this section was **re-measured today by live headless runs on this
> clone** — build `fa3e8a1a`, RTX 5070 Ti Laptop GPU, Newton `mujoco_warp`. Not prose,
> not `_scratch` memory, not commit-message headlines. Each deploy run set
> `OMNISIM_REQUIRE_NEWTON=1` and was confirmed by the engine line
> `world finalised (solver=MuJoCo (mujoco_warp, WorldInfo.newtonSolver))` (no silent ODE
> fallback). Telemetry logs live under `_scratch/verify/`. **Where this section disagrees
> with anything below or with any other doc/commit, this section is right.**
>
> ### The one finding that frames all of RL today
> The **only** policies that produce a **durable walk** in the real OmniSim Newton deploy
> are **Unitree's official re-hosted policies and behavior-clones (BC) of them.** Every
> **from-scratch / OmniSim-trained *humanoid* (G1)** controller — the residual walk, the
> ghost-built walk, the deterministic stand, the deterministic static-walk — hits the
> **same ~1.3–1.4 s forward-tip wall** in deploy. **Quadrupeds (OmniQuad/Go2/B2) DO walk
> durably on OmniSim-trained policies**, and there the learned residual is **load-bearing**
> (not a passenger — see correction #2). Humanoid deterministic **stands hold for H1 and
> Valkyrie but NOT for G1.**
>
> ### Table A — Durable walks: VERIFIED, 0 falls (real Newton deploy, today)
> | Robot | Policy | Provenance | Measured (this run) |
> |--|--|--|--|
> | **G1** | `g1_unitree_deploy/motion.pt` | **Unitree official, re-hosted** | **33.7 m, 0 falls**, z 0.763–0.775, 0.48 m/s, y −0.25 m |
> | **G1** | `g1_unitree_deploy/g1_bc_walk.pt` | **BC clone of Unitree** | **44.1 m @ 88 s, 0 falls**, min z 0.763, 0.50 m/s, y −0.49 m |
> | **H1** | `h1_unitree_deploy/motion.pt` | **Unitree official, re-hosted** | **30.0 m, 0 falls**, z 1.035–1.040, 0.42 m/s, \|y\|≤0.16 m |
> | **H1** | `h1_unitree_deploy/h1_bc_walk.pt` | **BC clone** (H1_LAT_KP=0) | **26.1 m @ 61 s, 0 falls**, z 1.03, 0.43 m/s |
> | **OmniQuad** | `gpu_omniquad_walk_vc_main` | **OmniSim-trained** (residual on trot) | **47.8 m, 0 falls**, bz 0.553, 0.32 m/s, y −0.55 m |
> | **Go2** | `gpu_go2_walk_main` | **OmniSim-trained** | **86.7 m, 0 falls**, 0.379 m/s, bz 0.29, y +0.25 m |
> | **B2** | `gpu_b2_walk_main` | **OmniSim-trained** | **110.7 m, 0 falls**, 0.492 m/s, bz 0.47, \|y\|≤0.08 m |
>
> (Distances are what each run reached in its wall-clock window, all still upright/forward
> at cutoff — they are lower bounds on durability, not fall points. None fell.)
>
> ### Table B — From-scratch / OmniSim-trained **G1** locomotion: NOT durable in deploy
> | Artifact | Provenance | Measured deploy result |
> |--|--|--|
> | `gpu_g1_walk15_c12` | OmniSim residual walk (the policy `run_g1_walk_deploy.ps1` wires) | **face-plants ~1.3–1.7 s** (forward fold, never advances) |
> | `g1_ghost_walk_v6.onnx` | **ghost-built**, full-authority RL, no Unitree weights (untracked) | **FALL@1.44 s** (v7 @ 1.04 s); moves *backward* (peak −0.24 m) |
> | `humanoid_static_walk` (G1) | deterministic quasi-static walk, untracked WIP | **FALL@1.42 s**; the +1.1 m "distance" is a fall-lunge + prone slide, not gait |
> | `gpu_newton_g1_walk_ft_pdoff_clamp` ("champion") | the doc's "+5.9 m / FALL@33.82 s" walker | **NOT IN THE REPO** — file absent, `_scratch` evidence gone, **unreproducible**. A surviving `_scratch/g1_deploy_c14.log` (different uncommitted policy) does show a real ~33 s bout (FALL@33.38 s, +12.9 m), so the *finite-bout class* existed — but no committed G1 policy reproduces it today. |
>
> ### Table C — Deterministic stands (pure-pose position hold, no RL)
> | Robot | Claim | Measured |
> |--|--|--|
> | **H1** | holds | ✅ **HOLDS** — bz 0.977, roll≈0, pitch −0.042, 0 falls (held full window) |
> | **Valkyrie** | holds | ✅ **HOLDS** — bz 1.104, roll≈0, pitch −0.010, 0 falls |
> | **G1** | "SOLVED, holds indefinitely" | ❌ **REFUTED — tips forward, FALL@~1.38 s** (steady-state collapse for 93 s; matches the `g1_stand_deploy.omniworld` world-header's own "MARGINAL, tips forward ~1.4 s on 6/7 runs"). *Caveat: deploy is a cold first-load; not re-tested warm — but corroborated by the world header and by the identical ~1.4 s wall hit by all four G1 deterministic controllers.* |
> | **Atlas** | negative (PPO ≈ baseline) | ❌ **CONFIRMED NEGATIVE** — trained 0.88 s vs zero-action 0.70 s survival; tips to −72° in <1 s; never stands; never deployed to Newton (the Newton stand is a separate analytic controller). |
>
> ### Physics train↔deploy parity (VERIFIED real, with one CI caveat)
> - `g1_golden_parity.py --structural` → **PASS.** Trainer (P1) ↔ canonical MJCF (P3) bit-clean
>   (≤2.1e-6); trainer ↔ deploy (P2) differs only in **representational** fields (fused-link
>   count 25 vs 34, SPAWN_Z bake) + the **gated** `OMNISIM_NEWTON_USE_LINK_COM` COM toggle.
>   All dynamics fields (inertia, gains, ranges, damping, friction, opt.*) identical; total
>   mass conserved 34.13 kg. The engine IS single-source — **no real, ungated physics gap.**
> - **`tests/test_g1_deploy_runtime_sync.py` — FIXED GREEN (2026-06-26).** It had drifted (the
>   `a340014c` welded compound-static inertia fix was never back-ported into the extract);
>   regenerated byte-faithful via `_gen_deploy_runtime.py`. Every extract-based parity proof again
>   rests on a faithful extract.
> - **BINARY-level parity now CERTIFIED (2026-06-26) — NEW.** Every proof above compared the trainer
>   against the Python *extract*, stepped in-process — never the real `omnisim-bin`. A deterministic
>   open-loop probe ([binary-parity-probe.md](binary-parity-probe.md)) runs the SAME scripted G1
>   sweep in the trainer Python AND the actual binary and diffs them. Chaos-free **welded lane: the
>   binary matches the trainer to ~1e-5 rad (median 4.7e-5 rad / 0.003°, float32 floor), base error
>   0 → PASS** — same physics to machine precision. (Free-base stand is looser, ~0.15°, purely from
>   inverted-pendulum chaos amplifying float32 round-off, not a model gap.) Deploy compiled robot
>   mass **28.03 kg = trainer exactly** (fixed-link fusion dynamically exact). The probe also FOUND
>   and FIXED a real engine bug: the `staticBase` robot-root weld pinned the base at the world ORIGIN
>   instead of its `.wbt` spawn pose (`47eed472`), affecting any staticBase robot spawned off-origin.
> - Open-loop G1 gait reference falls at **1.71 s** in plain MuJoCo → the RL policy is what holds
>   balance (as documented).
>
> ### Corrections this re-verification forces on the text below
> 1. **G1 stand is NOT solved** — it tips forward ~1.38 s. The "deterministic pure pose holds
>    indefinitely (`f48f00b7`)" headline does not reproduce on this build (it joins the doc's own
>    history of retracted G1-stand claims).
> 2. **OmniQuad's residual is NOT a passenger** — measured **+44 m with the policy vs max +0.13 m /
>    net −8 m backward bare** (no policy). On the current Newton build the learned residual is
>    *load-bearing* for forward locomotion. The "+5.03 m bare vs +4.87 m policy" figure is stale
>    (ODE-era / a different operating point).
> 3. **The durable G1/H1 walks are re-hosted/BC Unitree policies, not OmniSim-from-scratch.**
>    OmniSim's *own* G1 walk does not deploy durably (Table B). This is the real headline and the
>    real open problem.
> 4. **Go2 (+86.7 m) and B2 (+110.7 m)** exceed the doc's +66 m / +95 m figures.
> 5. **"No drift" is true for the *physics engine* (single-source, parity-clean) but NOT for the
>    docs or the CI extract:** doc-vs-reality drift (G1 stand, OmniQuad passenger, the missing
>    champion) and the red `g1_deploy_runtime` sync test are both live drifts found today.
>
> ---

---

# ⚠️ HISTORICAL (the 2026-06-19 → 2026-06-25 edition) — SUPERSEDED BY THE BANNERS ABOVE
## Retained for provenance. **Do NOT quote these numbers.** They are ~5 weeks stale.

> **Everything from here down to *[Drift-prevention protocol](#drift-prevention-protocol)* is the
> OLD edition of this doc, frozen in time.** It is kept as a gravestone — the claim→retract→narrower-
> restatement history is worth having — **but it is not the status of this project.** The canonical
> status is the **📍 WHERE WE STAND** banner at the top of this file. Where the two disagree, **the
> top banner wins**.
>
> Specifically, this block still contains, and you must NOT repeat:
> - Its own *"CANONICAL — start here … this is the single source of truth"* header (immediately
>   below). It **was**, on 2026-06-19. **It is not now** — that role belongs to the top banner.
> - **"No durable G1 walk / a ~1.3–1.4 s forward-tip wall"** — **superseded.** The G1 walks durably
>   in-engine via **Shadowing** (on the λ=0.9 balance harness — see **THE HARNESS** at the top).
> - **"Shadowing is the wrong architecture for the H1 walk"** (H1 row) — **superseded**: that verdict
>   judged the PRE-parity era (standalone trainer, unsolved deploy gaps, hand-designed kinematic
>   ghosts). Its *data* stands as history; its *conclusion* does not.
> - **A "Last updated: 2026-06-25" line and a "Latest 3 commits (2026-06-18)" section** — both frozen.
>   They are not the repo's latest anything.
> - Quadruped deploy distances, the deterministic-stand table, the Atlas negative result, and the
>   physics-parity work **remain broadly accurate** — they just predate the Shadowing/BATON/skill-library
>   era described above, and the quads have since been re-hosted on the **in-engine** trainer.
>
> ⛔ **Do not edit this block to "update" it.** If a fact here changes, update the top banner and
> leave the gravestone alone.

---

> 📍 **CANONICAL AS OF 2026-06-19 — ⚠️ NO LONGER CANONICAL (see the gravestone banner directly
> above).** This was the single source of truth for where
> OmniSim's RL work actually stood. **If any other doc, script comment, or commit
> message disagrees with this file, this file is right — fix the other one** (see
> *[Drift-prevention protocol](#drift-prevention-protocol)* at the bottom). The
> **G1** sections were re-verified **2026-06-19** against source, git, and **local,
> uncommitted** `_scratch/*.log` files (`_scratch/` is gitignored — these runs are
> *not* in-tree; a static audit — no live runs); OmniQuad/Atlas sections date to
> 2026-05-29.
>
> **G1 in one line (2026-06-19):** stands in deploy (deterministic **pure pose**,
> not RL); walks a **finite ~34 s** bout (learned residual) but has **no
> durable/indefinite walk**; ghost-similarity hits **84–88 % over a ~3 s window**
> but a **durable ≥80 % deploy walk is OPEN**; trainer↔deploy physics is
> single-source + CI-enforced. ⚠️ Older **"G1 walks +340 m / 297 m / 212 m, 0
> falls"** headlines are **trainer/old-path numbers that do NOT reproduce in
> deploy** — see the (now historical) *G1 — detail* section below. ⚠️ **2026-07-11: this whole
> one-liner is itself superseded** — the G1 walks durably in-engine via Shadowing (on the λ=0.9
> harness). Read the top banner.

> ✅ **NEW (2026-06-18) — G1 trainer ↔ deploy physics is now single-source AND verified.**
> The G1 Newton trainer and the OmniSim Newton deploy no longer hand-match physics: both
> derive their model from one place (`g1_physics.json` + `g1_physics_spec.py` + the prim
> URDF). Proven three ways — (1) a structural compiled-`MjModel` field diff shows **0 real
> physics gaps** between trainer and deploy (inertia, PD gains, ranges, damping, armature,
> joint/geom/opt fields all bit-identical) with `OMNISIM_NEWTON_USE_LINK_COM=1`; (2) a GPU
> golden trajectory (mjwarp) drifts only **8.5 mm** over the first 10 ticks between trainer
> and deploy; (3) a live H100 run's persisted `physics_config.json` **byte-matches the deploy
> spec on 11/11** checks. Same physics modulo the opt-in COM flag (default off) + documented
> representational / residual diffs. Full writeup:
> [g1-single-source-of-truth.md](g1-single-source-of-truth.md). (This supersedes the
> hand-matched parity the G1 rows below were written against.)

> ⚠️ **NEW (2026-06-18) — G1 ghost-similarity ≥80 % achieved OVER the walk window; NOT durable.**
> A per-joint similarity metric (`--eval-ghost-similarity`) showed the ambitious *human* ghost
> caps fidelity ~67 % (a balancing biped must deviate ~0.17 rad from a kinematic reference to stay
> up → RMSE floor). A **feasible** ghost — the robot's own gait phase-binned + L/R-symmetrised +
> extracted IN the Newton solver (`--build-achieved`, `--gait-style achieved`) — lifts the **shape**
> match to **FAIR all-13 84 % / moving 88 % over a 3 s window** (`gpu_newton_g1_walk_ACH2_pdoff`).
> **BUT an 18 s eval shows it topples ~6–8 s** (the 128/128 was a short-window artifact), and the
> deploy falls sooner. The achieved ghost was extracted in the trainer env where the champion lives
> only ~7.3 s (it walks 33.8 s in deploy) — the byte-matched model's per-tick drift **compounds** on
> the unstable biped. So **a durable ≥80 % deploy walk is OPEN**, entangled with the months-long G1
> trainer↔deploy durability gap. Full honest journal:
> [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md); recipe in
> [g1-deploy-walk.md](g1-deploy-walk.md).

**Last updated: 2026-06-25** (**H1 row updated 2026-06-25** with the executed
**pure-RL + closed-loop campaign** and its honest result: the closed-loop architecture
[obs frame-stacking + speed-regulating reward, commit `f7a6ac0d`] works, but **durability
is NOT solved — the best policy walks ~0.5 m and falls every ~1.7 s in the TRAINER
itself**, exposed by running the honest survival/distance eval instead of the auto-reset-
inflated reward curves; full journal [h1-walk-rl-journey.md §7](h1-walk-rl-journey.md).
Prior 2026-06-24: status-by-robot table extended to cover **H1,
Go2, B2**; **H1 row + method §2** record the
verified **deploy-physics fine-tuning REGRESSION** and the *"same solver ≠ matched
physics; the launch IC + obs pipeline must also match"* finding — see
[h1-walk-rl-journey.md](h1-walk-rl-journey.md); the OmniQuad/Newton-deploy row reconciled
to the 2026-06-23 "OmniQuad walks on Newton" finding; G1 sections re-verified 2026-06-19;
Atlas section dates to 2026-05-29). This is the single source of truth for *where OmniSim's RL work
actually stands today*. The per-robot / per-topic docs carry the deep journeys and recipes —
for **G1** see [g1-deploy-walk.md](g1-deploy-walk.md) (deploy recipe),
[g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md) (honest ghost journal),
[g1-single-source-of-truth.md](g1-single-source-of-truth.md) (physics spec),
[rl-two-layer-architecture.md](rl-two-layer-architecture.md) (the Ghost Method); for
others [atlas-stand-rl-journey.md](atlas-stand-rl-journey.md),
[omniquad-residual-rl.md](omniquad-residual-rl.md). **This doc carries the honest "is it
done?" answer** and the open gaps. If a headline elsewhere disagrees with this file,
this file is right — fix the headline.

It was written from a full read of the RL tree + a 21-agent verification
sweep that re-ran the key checks against the actual `_scratch/*.log` files
(and re-ran `build_g1_native.py` and the Atlas seed-lottery on this machine).
**Note on `_scratch/*.log` provenance:** `_scratch/` is **gitignored** — these
logs are **local, uncommitted runs**, not in-tree artifacts; treat them as
*regenerable-on-this-machine* evidence, not as committed provenance. Where a
number is *claimed but unverifiable from a committed artifact*, it is flagged as
such.

---

## TL;DR — status by robot (⚠️ HISTORICAL, 2026-06-25 — superseded by the top banner)

| Robot | In-sim (trainer) | OmniSim Newton **deploy** | Honest one-line verdict |
|--|--|--|--|
| **OmniQuad** (quadruped) | model+residual walks straight; velocity-conditioned walker (`gpu_omniquad_walk_vc_main`) trained on Newton-MuJoCo | **Newton: WALKS** — `gpu_omniquad_walk_vc_main` walks **+30 m straight, 0 falls** (chassis `bz`≈0.55) under Newton-MuJoCo via `run_omniquad_walk_vc_deploy.ps1` (2026-06-23). The earlier "~3 s collapse" was **largely an init bug** (a writable-stdio crash that silently fell back to ODE), now fixed — not a Newton dynamic-fidelity wall. Forced-ODE also walks straight (+1.43 m/8 s, z≈0.67). | **Walks on Newton now.** On the *older* model+residual policy the *learned residual is a passenger* (+4.87 m with policy vs +5.03 m with no policy at all; the hand-coded gait does the work). The newer VC walker is the working Newton deploy; the 2026-06-08 "W6 roll-collapse" characterization predates both the VC walker (2026-06-21) and the stdio fix (see OmniQuad — detail). |
| **G1** (23-DOF biped) | stand: pure pose holds; walk: residual walks ~7 s in trainer | **stand ✅ (pure pose) · walk ⚠️ finite ~34 s** — `run_g1_stand_deploy.ps1` holds the stand (06-10, `f48f00b7`); champion `ft_pdoff_clamp` walks **+5.9 m / `FALL@33.82 s`** then falls. **No durable walk.** | **Stand SOLVED via classical statics, NOT RL** (deeper-squat NOMINAL hip −0.30/knee 0.52 + analytic ankle PD off; the heavy-DR RL residual *destabilises* the stand ~2.4 s vs pure pose 12 s+). **Walk** is residual-driven but finite & fragile: ghost-similarity **84–88 % only over a 3 s window** (18 s eval topples ~6–8 s); the 33.8 s leaned on an **incorrect deploy COM** (corrected-COM `USE_LINK_COM=1` retrain falls ~11.7 s); a **durable ≥80 % deploy walk is OPEN**, entangled with the trainer↔deploy durability gap (champion 7.3 s trainer vs 33.8 s deploy). Trainer↔deploy physics is single-source + CI-enforced. Full breakdown in *G1 — detail* below. |
| **Atlas** (30-DOF biped) | PPO converges **to the analytic baseline** (μ≈0) | never run in Newton deploy; in-sim median 31–41 steps = a 0.5–0.65 s tip | **Negative result, shipped honestly.** The "trained" policy is the hand-tuned balance PD wrapped in PPO infra; PPO adds no measurable gain at 30 DOF. |

### Newer robot programs (added 2026-06-24)

These ship with on-disk trainers, policies, and Newton deploy worlds but were not in
the original OmniQuad/G1/Atlas table. They follow the **Shadowing** (Ghost) recipe — a
feasible kinematic reference + an RL residual that tracks it. Status verified against
the deploy worlds, run scripts, and commit history; **none has a *durable* humanoid
walk** — the durable-walk goal remains OPEN there too.

| Robot | Artifacts (verified on disk) | Newton **deploy** | Honest one-line verdict |
|--|--|--|--|
| **H1** (Unitree humanoid, 5-DOF legs) — **newest, current HEAD** | trainers [`gpu_mjwarp_h1_walk_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_h1_walk_trainer.py) (champion) + [`gpu_newton_h1_walk_trainer.py`](../../projects/policies/research/training/gpu_newton_h1_walk_trainer.py) (deploy-solver fine-tune); matched-physics MJCF [`h1_legs_newton.mjcf.xml`](../../projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml); ONNX residual wired into [`humanoid_walk_deploy.py`](../../projects/policies/research/controllers/humanoid_walk_deploy/humanoid_walk_deploy.py) (`HUMANOID_WALK_ONNX`); world [`h1_walk_deploy.omniworld`](../../projects/policies/research/worlds/h1_walk_deploy.omniworld); run script `run_humanoid_walk_deploy.ps1`. Commits `b82d53fa`..`da8b171a`. Full journal: [h1-walk-rl-journey.md](h1-walk-rl-journey.md). | **Walks a short ~2 s bout.** Deploy champion = **run 3** (`gpu_h1_walk_v3`, mjwarp-trained): **FORWARD +1.45 m at fall / +2.11 m peak, first-fall 2.03 s**; reproduces exactly (GUI + headless). The launch-settle OOD fix (`e4125fa6`, settle 0.3 s) was the enabler. **No durable H1 walk.** | ⚠️ **Phase-2 deploy-physics fine-tuning was RUN and REGRESSED the deploy** (`cf200cdc`, `da8b171a`): fine-tuning run 3 through the exact deploy `SolverMuJoCo` gives **1.58 s back** (fresh-URDF model) / **0.66 s back** (matched-MJCF model) — both worse than run 3's 2.03 s fwd. Verified twofold: (1) *same solver ≠ matched physics* — a fresh `add_urdf`+`add_ground_plane` uses newton's DEFAULT friction not the deploy's μ=2.0; loading the dumped MJCF via `add_mjcf` (`da8b171a`) lifted batched survival fidelity 1.26→1.85 s; (2) *even matched model+solver isn't enough* — run 3 and the fine-tune are byte-identical in the batched trainer (2.05 s/−0.72 m) yet the batched trainer walks **backward** where the deploy walks **forward**. Binding gap = the **launch IC** (deploy's 0.3 s settle lean + residual velocity, absent in the batched reset) + the **obs pipeline** (world-frame `getVelocity`/finite-diff qd vs trainer's exact MuJoCo-frame qvel), *not* the solver. Lever toward a durable 10-20 m walk = launch-IC + obs alignment, NOT fine-tuning. **⚠️ STRATEGIC COURSE-CORRECTION (2026-06-25):** the deeper diagnosis is that **Shadowing is the wrong architecture for the H1 walk** — a hand-designed *kinematic* ghost is not a balance solution (balance lives in reactive foot placement + push-off timing, absent from a joint-angle curve), and the bounded residual (res_scale 0.3) has too little authority *and* fights the ghost → it gives the natural gait but falls ~2 s, with no tuning that fixes it. The capture-point (DCM) foot-placement law (`726815ae`, `1cc72a69`) was the right idea but sagittal-only, and H1's fall went **lateral** (5-DOF legs, NO ankle-roll → weak lateral balance). **Decision: switch the H1 walk to full-authority RL + reward shaping (stability primary, fall-termination, naturalness as a SOFT term); keep Shadowing for feasible-reference non-continuous-balance motions (get-up, reach, sit-to-stand, toss-place). NEXT: train H1 pure-RL.** (⚠️ 2026-07-11: the original line here read *"on Modal H100"* — **the cloud path was REMOVED (`ef46a52e`); OmniSim trains LOCAL + IN-ENGINE**, [`projects/policies/training/`](../../projects/policies/training/) / `run_walk_rl.sh`. Never point at Modal/H100/cloud.) See [locomotion-shadowing-vs-pure-rl.md](locomotion-shadowing-vs-pure-rl.md). **▶ EXECUTED 2026-06-25 (pure-RL + closed-loop campaign, full journal [h1-walk-rl-journey.md §7](h1-walk-rl-journey.md)):** pure RL trains a walk after two fixes — anti-reward-hacking (it first *marched in place* at vx 0.12 → make velocity-tracking primary, survival cheap) and **closed-loop** (the naive policy was observation-INDEPENDENT / open-loop — a 1 rad/s qd perturbation moved the action <5%; fix = obs **frame-stacking** `--obs-history` + a speed-regulating `--overspeed` reward, commit `f7a6ac0d`; the policy is then genuinely feedback-driven + speed-regulating). **⚠️ HONEST RESULT: durability NOT solved.** Running the survival/distance eval (NOT the auto-reset-inflated reward/value curves, which read +0.87/+23 and HID the fall frequency) on the best policy (`runs/gpu_h1_purerl_cl3`): **~0.31 m/s, ~0.5 m (max ~2.1 m) before falling, falls every ~1.7 s IN THE TRAINER ITSELF**; deploy consistent (~0.3–0.7 m, <1 s). **Pure RL MATCHED Shadowing's ~2 m wall, did not break it.** Ruled out for the deploy gap (don't relearn): policy quality (closed-loop fails same), reward, qd (`H1_ENV_CORE` matched it), **CoM-forward pose** (the G1 cause — H1 statically stable, CoM 0.20 m behind foot front, long 0.28 m foot → G1's fatal 35 mm foot-shift harmless to H1), warmup phantom velocity (found+fixed `HW_NO_WARMUP=1`, +0.3 s, not root). **The wall is DURABILITY (sim AND deploy), not sim-to-deploy.** |
| **Go2** (Unitree quadruped) | policies `gpu_go2_walk_main` + `gpu_go2_walk_vc_main`; run scripts `run_go2_walk_deploy.ps1` / `run_go2_walk_vc_deploy.ps1`. | **Walks on Newton: +66 m, 0 falls** (OmniQuad residual stack retargeted to a Unitree quad; ~0.38 m/s). | Same recipe as the OmniQuad Newton walker; quadrupeds carry across Unitree quads cleanly. |
| **B2** (Unitree quadruped, large) | policies `gpu_b2_walk_main` + `gpu_b2_getup_main` (+ hill/stand checkpoints); run scripts `run_b2_walk_deploy.ps1` / `run_b2_getup_deploy.ps1` / `run_b2_hill_deploy.ps1`. | **Walks on Newton: +95 m, 0 falls.** Get-up (rise) also solved via Shadowing; hill-walk RL tracker is BLOCKED at the flat→ramp transition. | Walk + rise are real Newton deploys; the harder hill/transition motions are still open. |

**The single most important framing:** across all three robots, the *deployed
neural policy currently adds little-to-nothing over the hand-coded analytical
model.* That is consistent with the program's own meta-principle — *"RL value
scales inversely with analytical-model completeness"* — but it means today's
headline "RL successes" are mostly classical control. The place RL could
genuinely earn its keep (a stability-margin biped with no good analytical
baseline) is exactly where it is not working yet (G1 deploy).

---

## The "standard method" — what it actually is

Three distinct things get called "the standard method." They are at very
different maturity; keep them separate:

1. **Heavy-DR pure PPO on GPU `mujoco_warp`** — the documented *default*
   ([sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md),
   [`gpu_mjwarp_g1_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py)).
   Train to be *invariant* to the deploy-wrapper gap instead of matching it,
   plus 5 GPU speedups so heavy DR is tractable. **Status: works for
   quadrupeds and for in-sim training; empirically insufficient at deploy for
   stability-margin bipeds** (the G1 heavy-DR PPO residual *destabilises* the
   deploy stand — a deterministic **pure pose** is what actually stands, solved
   2026-06-10; a *heavier*-DR retrain deployed *worse*; Atlas mass-DR is a
   per-run lottery and was dropped). The recipe is a strong *starting point*,
   not a deploy guarantee.

2. **Train-in-the-deploy-solver** (the "faithful trainer") — the honest answer
   to #1's failure on bipeds: train PPO on the *exact* solver the deploy uses
   (`newton.solvers.SolverMuJoCo`), so the policy sees the real deploy
   dynamics. **Status: BUILT for H1, and the first verdict is a NEGATIVE
   result — it does NOT close the deploy gap.**
   [`gpu_newton_h1_walk_trainer.py`](../../projects/policies/research/training/gpu_newton_h1_walk_trainer.py)
   (commits `cf200cdc`, `da8b171a`) is a full GPU-batched (`SolverMuJoCo`,
   `use_mujoco_cpu=False`) trainer that warm-starts the H1 walk champion and
   fine-tunes it on the deploy solver (~98 k env-steps/s @ 1024 envs, laptop
   GPU). **Result (2026-06-24, verified): every deploy-solver fine-tune
   *regressed* the deploy** vs the mjwarp-trained champion (run 3: 2.03 s fwd →
   1.58 s / 0.66 s backward). Two lessons came out of it: (a) *same solver ≠
   matched physics* — the trainer must load the **dumped deploy model**
   (`add_mjcf(h1_legs_newton.mjcf.xml)`), not a fresh `add_urdf` build (newton's
   default friction ≠ deploy μ=2.0); (b) *even matched model + matched solver is
   insufficient* — the policy is byte-identical in the batched trainer yet
   diverges in deploy because the **launch IC** (deploy settle lean) and the
   **obs pipeline** (world-frame vel / finite-diff qd) differ. So this method is
   **not the lever it was hypothesized to be**; the lever is aligning the
   trainer's launch IC + obs to the deploy first. Full writeup:
   [h1-walk-rl-journey.md](h1-walk-rl-journey.md).
   (The older G1 foundation note: [`build_g1_native.py`](../../projects/policies/research/training/build_g1_native.py),
   commit `179a8d63`, builds a native-Newton G1-legs articulation that simulates
   faithfully under `SolverMuJoCo` with no NaN/explosion; a G1 trainer on top of
   it is still unwritten. The OmniQuad-only
   [`newton_solver_trainer.py`](../../projects/policies/research/training/newton_solver_trainer.py)
   is CPU-only `use_mujoco_cpu=True`, superseded for the GPU path by the H1
   trainer above.)

3. **The generic `--robot/--backend` registry**
   ([`robot_registry.py`](../../projects/policies/research/backends/robot_registry.py),
   [`train_robot.py`](../../projects/policies/research/training/train_robot.py),
   the `sb3`/`mjx`/`isaac`/`mujoco_warp` backends). Clean plumbing, but in
   practice **it only fully runs OmniQuad**: `sb3_cpu.py` hard-raises
   `NotImplementedError` for non-OmniQuad robots, no `robot_spec.json` sidecars
   exist for G1/Atlas, and the shipped G1/Atlas standing policies were produced
   by **standalone copy-paste trainer scripts that bypass the registry**.

**Bottom line on "standard":** what genuinely generalized is the *meta-principle*
and the *native-Newton-joint build pattern* (`omniquad_native.py` →
`build_g1_native.py`). "Heavy-DR is the default for new robots" is true as
**documentation guidance**, not as wired, runnable infrastructure.

**Agreed target architecture (2026-05-29):** the program is converging on a
**two-layer** standard — a deterministic controller that does the task by itself
(Layer 1) + a bounded RL residual that augments it for the unmodeled regime
(Layer 2). This is the deliberate, evidence-backed answer to the heavy-DR-pure-PPO
deploy failures. Full spec, interface contract, and phased plan:
[rl-two-layer-architecture.md](rl-two-layer-architecture.md). **Phase A (the
deterministic G1 balancer) is being validated now** — running log + findings +
resume steps: [rl-phase-a-validation-log.md](rl-phase-a-validation-log.md).
Interim (verdict PENDING): passive G1 topples ~1.1 s (forward, sagittal); no
position-mode ankle gain set has held yet; torque mode is the decisive next test.

---

## G1 — detail (⚠️ HISTORICAL — was canonical on 2026-06-19; **SUPERSEDED**)

> ⚠️ **This section is a gravestone.** It was the canonical G1 record on 2026-06-19 and the old
> drift rule sent readers here — **it no longer is, and it no longer does.** Its "no durable walk /
> ~1.4 s wall / stand-vs-RL" verdicts predate Shadowing, the in-engine trainer, and the whole
> 2026-07 skill library. **For any current G1 claim, read the 📍 WHERE WE STAND banner at the top
> of this file** (and **THE HARNESS** block, which tells you what the λ=0.9 crane carries).
> Kept below for the diagnostic lineage only.

> Scope: re-verified 2026-06-19 against source, git, and **local, uncommitted**
> `_scratch/*.log` files (`_scratch/` is gitignored — *not* in-tree; a **static
> audit** — no live training/deploy runs). Percentages and survival times come from
> those local logs / the per-robot journals; where no regenerable artifact backs a
> number it is flagged **documented-only**.

**Current verdict (one line):** G1 **stands** in deploy (deterministic pure pose)
and **walks a finite ~34 s bout** (learned residual), but there is **no
durable/indefinite walk**, and a **durable ≥80 %-ghost-similarity walk is OPEN** —
entangled with the trainer↔deploy durability gap.

### Stand — ⚠️ REFUTED 2026-06-26 (was claimed "✅ SOLVED, pure pose")

> ⚠️ **CORRECTION (2026-06-26, live re-run): this does NOT hold.** Running
> `run_g1_stand_deploy.ps1` unmodified (pure-pose fallback, deeper-squat nominal, ankle
> PD off) on build `fa3e8a1a`, G1 **tips forward and FALL@~1.38 s**, then lies face-down
> for the rest of the run (steady-state, not a transient). The `g1_stand_deploy.omniworld`
> world-header agrees ("MARGINAL … tips forward ~1.4 s on 6/7 runs"). The paragraphs
> below describe the *intended* fix; treat the "holds indefinitely" claim as **not
> reproducing on this build** (see the verified section at the top of this file).

- `run_g1_stand_deploy.ps1` holds the stand indefinitely in nominal conditions
  (roll≈0, pitch +0.04, bz 0.776, 0 falls; **documented-only** — the per-run stand
  log is regenerated, not committed), fixed **2026-06-10 (`f48f00b7`)**. Root cause
  was a **forward CoM + destabilising ankle PD**, *not* a sim2sim gap: deeper-squat
  NOMINAL
  (hip −0.30/knee 0.52) recenters CoM behind the foot front; the analytic ankle PD
  is **off by default**.
- ⚠️ **This is classical statics, not RL.** The heavy-DR PPO residual *destabilises*
  the stand (~2.4 s) vs the pure NOMINAL hold (12 s+). The shipped deploy default is
  the static pose (`G1_BALANCE_FALLBACK=1`).
- The old **"deploy stands to ~1.55 s then falls"** narrative is **pre-fix**
  (local `_scratch/g1_statics_on.log`, May 29 — uncommitted, gitignored) and is
  **superseded** by the 06-10 fix. (Diagnostic lineage preserved under *Historical*
  below.)

### Walk — ⚠️ a finite ~34 s deploy bout (residual-driven), NOT durable

- Best verified walker: **[`runs/gpu_newton_g1_walk_ft_pdoff_clamp/policy.onnx`] →
  +5.9 m, `FALL@33.82 s`** in the OmniSim Newton deploy
  (local `_scratch/g1_clamp_deploy.log` — uncommitted, gitignored). Landed
  **2026-06-17 (`9b6df709`)** after fixing a **silent XPBD fallback** (`cbe5e6f0`) +
  a trainer↔deploy joint-clamp parity gap. It is **finite, not indefinite.**
- ⚠️ **The 33.8 s walk leaned on an INCORRECT deploy COM** (COM-at-origin). With the
  now-correct parity flag `OMNISIM_NEWTON_USE_LINK_COM=1` the **same champion falls
  at ~11.7 s** ([`g1_physics.json`](../../projects/policies/research/backends/g1_physics.json)
  `_residuals`). The flagship 33.8 s is physics-incorrect-but-default; **a retrain on
  the corrected COM is the stated next step and has not been done.**
- ⚠️ The walk's balance is **the learned residual** (the analytic balance PD defaults
  OFF), so unlike stand the G1 *walk* residual is load-bearing — but the result is
  still only ~7–34 s.
- The old **"297 m / 212 m / +340 m, 0 falls"** numbers are **trainer/old-path
  headlines that do NOT reproduce in deploy**: the shape-c8 policy deploy-topples
  (`FALL@1.06s`, local `_scratch/g1_walk_shapec8_deploy.log` — uncommitted, gitignored).

### Ghost-fidelity ("walk ≥80 % like the ghost") — met over a window, OPEN durably

- Feasible "achieved" ghost (`runs/gpu_newton_g1_walk_ACH2_pdoff`): **FAIR all-13
  84.2 % / moving 87.9 % / sagittal 87.0 %, but over a 3 s window** *(documented-only)*.
  An **18 s eval topples ~6–8 s** (the "128/128 upright" was a short-window artifact).
- The ambitious **human/Winter ghost is a physical wall at ~67 %** — structural, not
  optimization: a balancing biped *must* deviate ~0.17 rad from a kinematic reference
  to stay up, flooring RMSE.
- ⚠️ The **champion lives ~7.3 s in the trainer Newton env vs 33.8 s in deploy** — the
  same byte-identical model (0 real-physics field diffs, **8.5 mm / 10-tick** drift),
  but the drift **compounds** on the inverted-pendulum biped. **A durable ≥80 % deploy
  walk is OPEN.** Honest journal:
  [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md).

### Physics — single-source train↔deploy (REAL, CI-enforced)

- Trainer and deploy derive their model from ONE place: `g1_physics.json` (knobs) +
  [`g1_physics_spec.py`](../../projects/policies/research/backends/g1_physics_spec.py) + the prim URDF
  (per-joint limits read **live**). `ACT_SCALE=0.3` is genuinely single-sourced.
- Enforced by
  [`tests/test_g1_physics_spec_conformance.py`](../../tests/test_g1_physics_spec_conformance.py)
  (18 tests pinning scalars + full leg-limit arrays) + CI
  [`.github/workflows/g1-spec-conformance.yml`](../../.github/workflows/g1-spec-conformance.yml)
  on every push to `projects/policies/**`.
- ⚠️ Caveats: the two **cross-consumer drift guards skip in the bare CI runner** (only
  `numpy`+`pytest` installed); the **GPU golden-trajectory check is a TODO**; the
  **mjwarp trainer still keeps literal `LEGS_JOINTS/NJ/OBS_DIM/NOMINAL`** (value-identical
  but not imported — a trainer-side ordering drift wouldn't be caught). Writeup:
  [g1-single-source-of-truth.md](g1-single-source-of-truth.md).
- **Stage-4 canonical MJCF** (`g1_23dof_omnisim.mjcf`) **landed + CPU-validated**
  (`2b0b5cb8`). Only the two *consumer switches* are deferred: trainer-from-MJCF (needs a
  retrain — **local + in-engine**, [`projects/policies/training/`](../../projects/policies/training/)
  via `run_walk_rl.sh`; the line here used to say "a Modal GPU retrain" — that cloud path was
  **REMOVED**, `ef46a52e`) and deploy-from-MJCF (needs a native rebuild, blocked by the local
  Qt5/Qt6 link failure).

### The journey in one paragraph (verified from git)

Registered 2026-05-27 (`3493cf69`) → stand solved in-sim 05-28 → a "deploy stand
SOLVED" claim made and **retracted the same day** 06-09 → genuinely fixed 06-10
(`f48f00b7`) → incremental walks 06-10/11 → kinematic "ghost" target 06-12 → shape-c8
**"297 m zero falls"** 06-13, **corrected 06-14** as non-reproducing (deploy `FALL@1.06s`)
→ Ghost Method named 06-16 → **real Newton deploy walk 5.9 m / 33.8 s** 06-17 (`9b6df709`)
→ single-source physics verified 06-17/18 → ghost-fidelity 84–88 % over a window 06-18.
The recurring pattern is **claim → retract → narrower honest restatement**; the latest
journals own it.

### Latest 3 commits (2026-06-18)

- `3bd26287` **`--vx-cmd-max`** — +7 lines; wires an *already-existing* vx-conditioning
  mechanism (`vx_cmd=0` freezes the gait phase → stand) into the Newton trainer `main()`.
  Default `0.0` = off.
- `ab689367` **`--eval-scripted`** — +82/−1, **eval-only**; prints per-segment
  stand/walk/stop **FAIR ghost-similarity + durability**. ⚠️ The per-segment metric is
  **FAIR amplitude only — NOT SHAPE/Pearson** (the 84–88 % "shape" number comes from a
  *different* eval path); it requires a vx-conditioned policy that **does not exist as a
  trained artifact yet**; nothing in deploy/tests/docs wires to it.
- `7deb93fa` **honest journal** — docs-only; adds `g1-ghost-fidelity-journey.md`,
  downgrades over-claims in 5 docs + AGENTS.md.
- Read together they set up a **"stand-first → scripted stand/walk/stop" hypothesis**
  with tooling — *not* a demonstrated result.

### Deploy how-to (so agents don't chase dead paths)

- **Controller:** [`g1_walk_deploy.py`](../../projects/policies/research/controllers/g1_walk_deploy/g1_walk_deploy.py)
  drives BOTH stand and walk worlds. (It still prints `[g1_stand_deploy]` and its
  docstring says "standing" even when walking — cosmetic, not a bug.)
- **Canonical launcher:** [`g1_deploy_launch.py`](../../projects/policies/research/runners/g1_deploy_launch.py)
  (spec-driven; default `ft_pdoff_clamp` + `g1_walk_arms_deploy_mjwarp.omniworld`; `--print-env`).
- **Env flags:** `OMNISIM_NEWTON_STATICS=1` + `OMNISIM_NEWTON_SUBSTEPS=4` (needed just to
  step at all); `G1_DEPLOY_LOG=<path>` for telemetry (falls print `FALL@<t>s`). There is
  **no `--minimize`** flag; headless is via `scripts/dev/headless_runner.py`.
- ⚠️ **Stale entry point:** `projects/policies/research/runners/run_g1_walk_deploy.ps1 -Arms` points at the
  shape-c8 policy with a "297 m zero falls" label — that policy **deploy-topples ~1 s**.
  Prefer `g1_deploy_launch.py`.
- ⚠️ Deploy intermittently hits a **cold-load crash (exit 1, no telemetry)** — root cause
  unidentified.

### Trainer note

- **No champion / best-checkpoint selection exists** — the trainer saves only the *final*
  policy after all iterations. "Champion" in the docs means a **hand-picked warm-start**
  (`ft_pdoff_clamp`), not an automated best-of-run.

### Open problems (G1)

1. **Durable (indefinite) ≥80 %-ghost deploy walk** — the headline goal; OPEN.
2. **Trainer↔deploy durability divergence** (7.3 s vs 33.8 s on a byte-identical model)
   — asserted as compounding drift, **not root-caused**.
3. **Retrain on corrected COM** (`USE_LINK_COM=1`) — champion drops to ~11.7 s; no
   corrected-physics policy exists yet.
4. **Cold-load crash** (exit 1) — unfixed; blocks honest deploy judgement.
5. **Stage-4 MJCF consumer switches** — deferred. Both are still OPEN: the trainer-from-MJCF
   switch needs a **local, in-engine retrain** (`projects/policies/training/`, `run_walk_rl.sh`)
   and deploy-from-MJCF needs a native rebuild. (This line used to read "Modal GPU retrain" —
   the cloud path was **REMOVED**, `ef46a52e`.)

---

### Historical (superseded): the pre-2026-06-10 stand-debugging journey

> ⚠️ **The following is the historical debugging record for why the deterministic
> stand was hard.** The stand was **SOLVED 2026-06-10** (see *Stand* above) and the
> G1 now also walks a finite bout; these paragraphs describe a state that **no longer
> holds** and are kept only for the diagnostic lineage. Do not cite them as current.

**Deploy requires two env vars just to step at all** (post the 2026-05-29
chassis-freeze fix `d56cbf5`):
`OMNISIM_NEWTON_STATICS=1` (gives the forced-`SolverMuJoCo` model a ground to
stand on) and `OMNISIM_NEWTON_SUBSTEPS=4` (a single 16 ms contact solve
NaN-explodes at first foot contact). Without `STATICS`, the world finalises
but never steps.

**Two cheap levers are empirically ruled out:**
- **More DR** — a heavier-DR retrain hit ≈98.6 % in-sim but deployed *worse*:
  `FALL@1.47s` + a contact explosion to `bz≈1.1e5 m`
  (local `_scratch/g1_contactdr_deploy.log` — uncommitted, gitignored).
- **Ground friction** — μ = 1.0 / 1.5 / 2.0 all give the *identical*
  `FALL@1.55s` (local `_scratch/g1_mu1.5.log` / `_scratch/g1_mu2.0.log` —
  uncommitted, gitignored). If the feet were slipping the
  failure time would move with μ; it doesn't.

**Root cause (the key recent insight, milestone-1, `179a8d63`):** a passive
NOMINAL-hold under the *exact* deploy solver
([`build_g1_native.py`](../../projects/policies/research/training/build_g1_native.py)) also
topples at ~1.5 s (independently reproduced on this machine: z-collapse
~1.63 s, no explosion). So the 1.55 s gap is **largely inherent
inverted-pendulum instability the policy must actively overcome**, not purely
`mjw.step ≠ SolverMuJoCo.step` wrapper drift. *Caveat:* the stronger reading —
"the deploy policy's active balance is nearly ineffective" — is **not yet
established** (it ignores the deploy's 0.5 s settle window; the passive topple
is substep-sensitive while the deploy 1.55 s is substep-*invariant*; and the
deploy adds a contact explosion the passive case lacks). Treat "inherent
instability exists" as confirmed and "policy barely beats nothing" as a working
hypothesis.

**Fix path:** train inside the deploy solver (method #2 above). Foundation is
built + verified; the trainer is the next dedicated effort.

**FINDING (2026-06-09 — harness + deploy-verified, see [rl-phase-a-validation-log.md](rl-phase-a-validation-log.md)
§"Session 2026-06-09"):** the 1.55 s collapse is **partly** the deploy's too-soft
joint PD. In the legs-only native-Newton harness, a passive NOMINAL hold at
**ke=400/kd=60 holds ≥30 s with zero policy** (vs 1.12 s at ke=20); naive PD balance
laws (position/torque, ankle±hip) at ke=20 all fail <1 s (ankle CoP-saturated).
**BUT on the real `g1_stand_deploy.omniworld` (zero policy), ke=400 only gets 0.51 s →
1.44 s** — clean & level through ~1 s, then a forward topple; ke=800 too stiff (0.51 s).
NB the deploy world loads the SAME `g1_legs_omnisim.urdf` as the harness (legs-only,
~28 kg — NOT the full 34 kg robot), so the gap is NOT arms/CoM. Diagnosed: ankle
effort (35 vs 88) REJECTED, legacy analytic PD REJECTED (pure-NOMINAL deploy still
falls 1.70 s) — the real difference is a **FORWARD DRIFT (bx→1 m) present only on the
deploy**, absent in the direct-Newton harness at the identical robot/solver/ke/command.
⚠️ A `GROUND_MU=2.0` "STANDS INDEFINITELY (5527 s)" claim was made and **RETRACTED** —
it was ONE non-reproducible run; on re-test the same config tips forward at ~1.4 s on
~6/7 runs. The failure is a forward TIP (feet PLANTED — friction is NOT it): the deploy
spawns straight-legged and FOLDS into the squat during settle, and the marginal ke=400
ankle can't recover. Decisive proof it's the deploy MODEL: the harness's `add_urdf` builds
a **23-body** articulation that stands; the deploy's `OmNewtonBackend` import builds a
**~14-body** one (verified via `OMNISIM_NEWTON_SAVE_MJCF` dump) that is unstable at NOMINAL
even when spawn-seeded. So deterministic standing is NOT achievable on the deploy model.

**RL attempt (Option 2, 2026-06-09):** dumped the EXACT deploy MuJoCo model
(`OMNISIM_NEWTON_SAVE_MJCF` → renamed via `import_newton_mjcf.py`) and trained heavy-DR
PPO **on the deploy model** (`gpu_mjwarp_g1_stand_trainer`, 600 iters/29.5 M steps,
mass±0.45/fric±0.6/kp±0.5/push 2.5 m·s⁻¹@6%/latency5/init-q±0.20; policy at
`runs/gpu_g1_deploy_robust/`). **The policy does NOT transfer to deploy** — FALL@1.15 s
(policy-only, *worse* than the 1.70 s deterministic baseline) / FALL@2.02 s (policy +
`G1_RAMP_S=1.5` gentle handover). The policy being *worse* than no-policy signals a
**trainer↔deploy obs MISMATCH** (top lead): the trainer's angular velocity is MuJoCo
free-joint `qvel[3:6]` = **body frame**, but the deploy controller feeds `getVelocity()[3:6]`
= **world frame** (`g1_stand_deploy.py`); also check the `proj_gravity` row-vs-column
convention. Fixing the deploy obs to match the trainer (rotate world ang-vel into body
frame) may make the EXISTING policy work with no retrain. If not, add base-pitch/velocity
+ straight-leg(fold) jitter to the trainer reset (it currently resets to NOMINAL±joint-jitter,
never the folded handover state) and retrain. **Resumable artifacts:** deploy-model MJCF at
`_scratch/g1_deploy_model_ke400.mjcf.xml`, raw dump `_scratch/g1_deploy_dumped.mjcf.xml`,
policy `runs/gpu_g1_deploy_robust/`.

**UPDATE 2026-06-10 — obs-frame fix + fold-init retrain, still NOT standing.**
(1) **Obs-frame bug FOUND + FIXED + committed** (4398d3e9): verified MuJoCo free-joint
`qvel[3:6]` = BODY frame / `qvel[0:3]` = WORLD (proj_gravity already matched). The deploy
controller fed WORLD-frame ang-vel; now rotates to body (`R^T·ω`). Real correctness win for
ANY policy — but the existing policy STILL fell (~1.1 s, now via roll, not forward). (2)
**Fold-init retrain**: added base-tilt + base-velocity init DR to the trainer (commit
44517d9c, `--dr-init-tilt-band/-vel-band`), retrained on the deploy model (34.4 M steps,
tilt 0.35/vel 0.4 + heavy DR, `runs/gpu_g1_deploy_robust2/`). Deploy-test: **FALL@0.82 s**
(now backward). So across buggy-obs/fixed-obs/fold-init the policy is a POOR deploy
balancer despite ~0.92 ep_rew/step IN training → a residual train↔deploy gap remains:
prime suspects = the deploy's **joint-qd low-pass smoothing** (`qd_alpha` in `g1_stand_deploy.py`)
vs the trainer's exact `qvel`; the **baseline roll/pitch-rate source** (deploy world ang-vel
components vs trainer finite-diff); the per-step **wrapper** (control latency/state-sync) not
fully covered by DR; and possibly the documented **"net learned value ≈ 0 at deploy"** problem
(the policy may be a marginal balancer). NEXT (multi-session): verify the policy's TRAINING
survival explicitly (not just ep_rew); align the deploy obs/baseline EXACTLY (drop qd smoothing,
finite-diff roll/pitch-rate) to the trainer; consider a reward/curriculum pass; then iterate.
**Net: G1 deploy stand NOT solved (deterministic, RL cycle 1, obs-fix, fold-init retrain all
tried). The obs-frame fix is a kept correctness win; the rest is documented multi-session work.**

**Also open (at the time):** walking via
[`g1_model_walk.py`](../../projects/policies/research/controllers/g1_model_walk/g1_model_walk.py)
was then an open-loop CPG with **no policy**. ⚠️ **This no longer describes the
walk** — G1 now walks in deploy with a learned residual policy via
[`g1_walk_deploy.py`](../../projects/policies/research/controllers/g1_walk_deploy/g1_walk_deploy.py)
(`g1_model_walk.py` survives only as the open-loop baseline; see the canonical
*Walk* section above). The earlier arms-reproducibility gap (`g1_full.mjcf.xml`
missing) was later closed by the Stage-4 canonical MJCF work (`g1_23dof_omnisim.mjcf`).

*(End of historical record.)*

---

## Atlas — detail

**State:** the whole G1 pipeline ports (after three mandatory Atlas-specific
env fixes), and a deployable `policy.pt`/`policy.onnx` exists — but **PPO never
learned anything that beats the analytic baseline.** A live head-to-head on
this machine (identical seed, 512 envs): trained policy vs zero-action median
survival = **41 vs 41 / 31 vs 31** — zero improvement. The policy μ stays
pinned at ~0 through all 200 iterations.

**Two findings worth keeping (both reproduced live):**
1. **Mass DR in `mujoco_warp` is per-*run*, not per-*env*** (single shared
   model). Only **2 of 8 mass seeds** leave even the baseline standing — it's a
   lottery, so it was dropped for the deliverable.
2. **A near-saturating analytic baseline + heavy DR + 30-DOF gradient noise
   starves PPO of signal.** The baseline already harvests ~88 % of the per-step
   reward ceiling at iteration 1, so μ has no profitable direction to move.

**Caveats:** "standing" oversells it — median 31–41 steps ≈ 0.5–0.65 s is a
slow tip, not a stand; "97 % cumulative survival" is episode-cycling
(reset-on-fall), not continuous standing. It was **never deployed into Newton**
(no Atlas deploy log exists). The registered spec
(`atlas_robot_spec.py`, deleted with the rest of the Atlas package) was a
*stale walking spec* with the abandoned deep-squat NOMINAL — the shipped
standing pose/scale live in the trainer + deploy controller, not the spec (now
flagged in-file).

---

## OmniQuad — detail

> **UPDATE 2026-06-23 — OmniQuad DOES walk on Newton** (the TL;DR row above now reflects this;
> the original "Newton: COLLAPSES" wording was reconciled 2026-06-24).
> Verified on the velocity-conditioned walker (`gpu_omniquad_walk_vc_main`) under Newton-MuJoCo
> (engine log `world finalised (solver=MuJoCo (mujoco_warp))`): chassis `bz` holds ~0.55, walks
> +30 m straight, 0 falls (Go2 +66 m, B2 +95 m likewise). The earlier "collapse in ~3 s" was
> **largely an init bug**, not a Newton dynamic-fidelity wall: warp's startup banner wrote to a
> `None`/closed `sys.stdout` under the headless `DEVNULL`-stdout launch, the Newton FFI smoke
> (`newton.ModelBuilder()`) raised `'NoneType' object has no attribute 'write'`, and the engine
> **silently fell back to ODE** — on which these Newton-tuned worlds collapse. Fixed in
> `OmNewtonBackend.cpp` (writable-stdio guard before the warp import); Newton now initializes
> reliably and the quadrupeds walk. The 2026-06-08 "W6 lateral/roll collapse" characterization
> predates both the 2026-06-21 VC walker and this fix. (The model+residual notes below remain
> historically accurate for the older policy.)

**State:** the model+residual recipe (gait engine + IK + balance PD + tiny
learned residual) is real, shipped, and verified. Trains in ~84 s / 50 k steps
under Newton; all 5 checkpoints deploy without falling.

**The honest catch (stated in the doc, worth repeating here):** under Newton
the trained residual is a **passenger** — the open-loop model walker alone is
**+5.03 m** straight, and the 50 k-step policy on top is **+4.87 m** (marginally
*worse*). The deployed neural net is functionally cosmetic; the hand-coded gait
is what walks.

> ⚠️ **CORRECTION (2026-06-26, live re-run): the "passenger" framing no longer holds on
> the current build.** Running `run_omniquad_walk_deploy.ps1` with the policy vs `-Bare` (no
> policy): **+44.0 m @ 0.40 m/s with the policy** vs **max +0.13 m / net −8 m backward
> bare**. The bare trot gait stays upright (bz≈0.54, 0 falls) but makes *no forward
> progress*; the learned residual is what converts it into forward walking — it is
> **load-bearing, not cosmetic**. The +5.03 m bare figure above is stale (ODE-era / a
> different gait operating point). (The VC walker `gpu_omniquad_walk_vc_main` walks +47.8 m,
> 0 falls — see the verified section at the top.) The recipe's real, durable contribution is *negative knowledge*:
it taught the team when residual RL is pointless (analytical model already at
the limit) — the lesson that became the meta-principle.

*Unquantified:* push-recovery is named as "what justifies the RL part" and has
trained `perturb` policies on disk, but no survival-under-push delta vs the
no-policy walker is reported anywhere — so whether the residual ever earns its
keep on OmniQuad is, as of today, unproven.

---

## Cross-cutting findings

- **Net learned value delivered to deploy is small-to-negative** across all robots
  (OmniQuad residual is a *passenger*; Atlas policy *== baseline*; G1 **stand** is pure
  statics — the RL residual makes it *worse*). The one place an RL residual is now
  **load-bearing** is the **G1 walk** (analytic balance off → the residual does the
  balancing), but it still yields only a **finite ~7–34 s** bout, not a durable walk.
  The one clean learned win remains the OmniQuad *ODE* residual. ⚠ 2026-08-08: that result's
  configuration no longer exists — `bdc02139` deleted the ODE backend, so the OmniQuad ODE
  residual cannot be re-run, and this "one clean learned win" is now an unrepeatable
  historical datum rather than a standing result.
- **"Passenger" vs. "saboteur" — a complete baseline on a *marginal static* task makes
  the correction layer net-NEGATIVE, not net-zero** (2026-06-23, stand-and-hold-cubes
  demo). A residual on a *statically-stable* task with redundancy (OmniQuad walk, 4 contacts)
  is a harmless passenger; a residual on a *2-contact inverted-pendulum stand* (G1)
  **self-topples a stationary robot in ~1.3 s with no push** — authority itself reopens
  the instability the stiff hold closes, and the sim-to-deploy gap mis-times the feedback
  loop with no kinematic redundancy to absorb it. The effect is **not RL-specific**: even a
  hand-coded reactive ankle lean is load-bearing for the marginal G1 but topples the stiff
  H1 (~6 s) / Valkyrie (~18 s) that hold all cubes passively. **Rule: on a static-balance
  task, gate any correction layer (learned or hand-coded) on a measured per-robot delta
  over the bare hold; if the bare hold passes, ship it and add nothing.** Mechanism +
  evidence: [rl-two-layer-architecture.md §3.8](rl-two-layer-architecture.md#38-stand-and-hold-cubes--passenger-vs-saboteur-and-why-a-residual-on-a-static-stand-goes-net-negative-2026-06-23);
  demo: [humanoid-deterministic-stand.md](humanoid-deterministic-stand.md).
- **DR is double-edged.** The exact lever that "closed" the in-sim gap *hurts*
  deploy: G1 more-DR → contact explosion; Atlas mass-DR → per-run lottery,
  dropped. The recipe's central thesis is empirically walked back for *both*
  humanoids.
- **Deploy-physics fine-tuning does NOT close the deploy gap — and *matched
  solver ≠ matched physics ≠ matched deploy*** (2026-06-24, H1 walk; verified).
  Training a policy through the *exact* deploy solver (`SolverMuJoCo`) and even
  on the *exact* dumped deploy model **regressed** the H1 deploy every time
  (champion 2.03 s fwd → 1.58 s / 0.66 s backward). Two compounding reasons,
  each a reusable lesson: **(1)** a fresh `newton add_urdf` + `add_ground_plane`
  build silently uses newton's *default* friction, not the deploy's μ=2.0 — you
  must train on the **dumped deploy model** (`add_mjcf`), the H1 analogue of the
  G1 "single source of truth" rule. **(2)** Even at byte-level model+solver
  parity the policy is *byte-identical in the batched trainer yet diverges in
  deploy* (the batched trainer walks **backward** where the deploy walks
  **forward**) — because the **launch initial condition** (the deploy's settle
  lean + residual velocity) and the **observation pipeline** (world-frame
  `getVelocity`/finite-diff qd vs the trainer's exact body-frame qvel) are *not*
  matched. A batched-trainer metric can be a poor proxy for a single-robot
  deploy launch even with identical physics. Full evidence:
  [h1-walk-rl-journey.md](h1-walk-rl-journey.md).
- **The "standard method" is documentation, not infrastructure** (see method
  §3). The registry runs OmniQuad; humanoid policies came from standalone scripts.
- **Almost everything rests on un-re-run `_scratch` logs and self-reported
  prose.** The headline G1 walk/ghost numbers (the **84–88 % shape over a 3 s
  window**, the **7.3 s trainer / 33.8 s deploy** split, the **8.5 mm / 10-tick**
  drift) are **documented-only** — they come from session prose / per-run logs, not
  regenerable committed artifacts. ⚠️ **The `_scratch/*.log` deploy logs cited
  throughout this doc are NOT committed or in-tree** — `_scratch/` is **gitignored**
  (`git check-ignore` confirms), so the `FALL@33.82s` / `FALL@1.06s` / `FALL@1.55s`
  numbers come from **local, uncommitted runs on this machine** (some of which are no
  longer even on disk), not from versioned artifacts. `build_g1_native.py`'s
  "~1.5 s topple" is a *hardcoded print string*, not a measured value. Per the repo's
  own headless-PASS rule, treat un-logged percentages as claims, not measurements.

---

## The post-fall contact explosion + the guard added in this pass

**Symptom:** once G1 tips at ~1.55 s, the deploy contact solve drives the
pelvis `bz → 1e4–1e5 m` (every run, even at `SUBSTEPS=4`).

**Root cause:** the post-step clamp in
[`OmNewtonBackend.cpp`](../../src/omnisim/physics/OmNewtonBackend.cpp) only
bounds **articulated joint DOFs** (`joint_q`/`joint_qd`). The **floating-base**
`body_q`/`body_qd` are *not* joint DOFs, so a large contact impulse on the base
(a biped tipping, or a high-speed collision) diverges the base position with
nothing to catch it.

**Fix added (this pass):** a **base-divergence guard** in `OmNewtonBackend`'s
step path — after the solver step it checks `body_q` for non-finite values or
base coordinates beyond `OMNISIM_NEWTON_BASE_GUARD_MAX` m (default 1000) and,
on divergence, freezes the articulation at its last finite/in-bounds pose and
zeros velocities. It is a **strict no-op for any physically-valid state**, so it
cannot change a healthy sim's determinism, and it is **default ON** (set
`OMNISIM_NEWTON_BASE_GUARD=0` to disable). It also helps the husky
high-closing-speed NaN case.

**⚠️ NOT YET VERIFIED.** The change is embedded Python inside the C++ Newton
backend, so it **needs a binary rebuild** (`scripts/dev/build_with_cd.sh`, from
Bash) **+ a deploy run** (from PowerShell, with `OMNISIM_NEWTON_BASE_GUARD=1`
on [`g1_stand_deploy.omniworld`](../../projects/policies/research/worlds/g1_stand_deploy.omniworld)) to
confirm the explosion is suppressed without regressing healthy worlds. Until
then it is landed-but-unproven. Note this is a *graceful-failure* fix (stops the
numerical blowup); it does **not** make G1 stand past 1.55 s — that's method #2.

---

## Open issues / prioritized backlog

1. **(done this pass)** Stale "G1 stands forever / 44+ s" headlines corrected
   across `README.md`, `docs/developer/README.md`, `humanoid-balance-gap.md`,
   the playbook bottom-line, the canonical-template trainer docstring, and
   `omniquad-residual-rl.md`; `sim-to-deploy-rl-recipe.md` given a scope caveat.
2. **(landed, needs verify)** Base-divergence guard for the contact explosion
   — rebuild + deploy-run to confirm.
3. **Decide on the faithful trainer (method #2).** Multi-session. First *prove*
   the cpu-vs-mjwarp faithfulness on a hold-NOMINAL test before building the
   full PPO trainer; throughput (~1456 steps/s single-env) is the main risk.
4. **Reproducibility holes:** commit `g1_full.mjcf.xml` (or its dump inputs) so
   the arms policy is rebuildable; capture and commit eval logs (the cited
   98.6 % G1 eval has no surviving log — `g1_retrain.log` is 2296 `nefc overflow`
   warnings with no survival line); generate `robot_spec.json` for G1/Atlas or
   document that generic deploy is OmniQuad-only.
5. **Reconcile perf claims** (see below).
6. **Stale-spec cleanup** (`atlas_robot_spec.py` walking spec, dead
   `eval_atlas_*` tools) — flagged in-file; blocked from deletion by the
   migration's no-rename freeze, so left annotated.

---

## Perf & reproducibility caveats

- **Hardware/throughput:** the headline `~132 k env-steps/s` / `3 min 43 s for
  30 M-step PPO` / **RTX 5070** figures appear in the playbook + READMEs, but
  the repo's documented dev box is an **RTX 3060 Laptop**, and a verifier's
  re-runs on this checkout measured **~27–62 k env-steps/s** (a 29.5 M-step run
  took ~7.9 min). Either the 5070 numbers are from a different/original training
  box or they're aspirational — **the author should confirm which**. This
  matters because "DR is only feasible because training is cheap" is load-bearing
  for the recipe, and on this hardware the run is ~8 min, not <4.
- **Unverifiable-from-artifact numbers:** the G1 "98.6 % / 2466-of-2500" eval,
  the OmniQuad "+4.87 m straight" deploy, the OmniQuad from-scratch v3/v4 failures, and
  the build_g1_native milestone-1 topple time all lack a committed log. They are
  reported in good faith in docs/commit messages but cannot be regenerated from
  the tree today.

*(End of the historical 2026-06-19 → 2026-06-25 record. Everything below is CURRENT.)*

---

## Drift-prevention protocol

**This file is canonical. Keep it that way.**

1. **One source of truth.** This file holds the honest "is it done?" answer for RL.
   If a README, journey doc, script comment, or commit message disagrees, **this
   file wins — fix the other one.** Do not create a new "current status" doc; update
   this one.
2. **Before claiming ANY robot result, read the 📍 WHERE WE STAND banner at the TOP of
   this file** (AGENTS.md points here) — **not** the *G1 — detail* section, which is inside
   the gravestoned historical block and is frozen at 2026-06-19. Three distinctions the
   banner exists to protect: "stands" ≠ "stands via RL"; "walks" ≠ "walks durably"; and
   **"walks" ≠ "walks unassisted"** — every shipped G1 walk / turn / carry runs on the
   **λ=0.9 balance harness** that carries ≈2× body weight (the stair climb is the one demo
   with the vertical wire off). A **trainer/batched-eval number is never a deploy result** —
   say which one you are quoting.
3. **Every number cites an artifact** — a commit hash, a test, or a **local,
   gitignored** `_scratch/*.log` run — or is tagged **documented-only**. ⚠️ A
   `_scratch/*.log` is **not committed/in-tree** (`_scratch/` is gitignored), so it is
   *regenerable-on-this-machine* evidence, not versioned provenance; the only truly
   committed artifacts are commits and tests. Un-logged percentages are claims, not
   measurements — the repo's headless run prints `PASS` on load, not on success.
4. **When robot state changes,** update (a) the **top banner** + its date, (b) the
   ***SHIPPED DEMOS — canonical adjudications*** block (if a shipped demo is affected), and
   (c) the relevant deep-dive doc — in the same change. **Never edit the gravestoned
   historical block to "update" it** — it is provenance, not status.
5. **If a demo ships publicly, it gets an entry HERE.** The stair climb, box delivery, the
   90° turn and the in-engine quad trainer all shipped with **no canonical entry** (added
   2026-07-11). A public claim with no adjudication in this file is the drift.
6. **Known stale sources — do NOT trust their headlines (correct them when seen):**
   - **The historical block above** (2026-06-19 → 2026-06-25) — gravestoned; every per-robot
     "wall" in it predates Shadowing. Read the top banner instead.
   - `rl-journey.md` §11 status table and `g1-walk-rl-journey.md` — "+340 m / 297 m
     / 212 m, 0 falls, shipped" are trainer/old-path numbers; deploy is finite (33.8 s).
   - `run_g1_walk_deploy.ps1 -Arms` — "297 m ZERO falls" label; that policy
     deploy-topples ~1 s. Prefer `g1_deploy_launch.py`.
   - `rl-phase-a-validation-log.md` §10 "5527 s STANDS INDEFINITELY" — **RETRACTED**
     (non-reproducible).
   - Any "G1 stands forever / 44+ s" headline — the stand is a finite-margin pure-pose
     hold, durable in nominal conditions but not an RL achievement.
   - **Anything pointing at `cloud/`, Modal, or an H100 for training.** The cloud wrappers
     were **REMOVED** (`ef46a52e`); OmniSim trains **local + in-engine** by policy
     ([`projects/policies/training/`](../../projects/policies/training/), `run_walk_rl.sh` /
     `run_quad_walk_rl.sh`). Never re-add a cloud path.

---

## Deep-dive docs

**G1 (current):**
- [train-deploy-gap.md](train-deploy-gap.md) — **synthesis + recipe: the two gaps
  (pipeline-parity vs durability), the enumerated + re-verified divergence table, and
  Unitree's proven deploy recipe** as the durability answer. Routes to the owners
  below; defers all status to this file.
- [g1-deploy-walk.md](g1-deploy-walk.md) — the deploy walk recipe + honest status
  (5.9 m / 33.8 s; ghost ≥80 % not durable).
- [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md) — the honest
  ghost-similarity journal (the ~67 % human wall, the 84–88 % feasible ghost over a
  window, the durability correction).
- [g1-single-source-of-truth.md](g1-single-source-of-truth.md) — the trainer↔deploy
  single-physics-spec work + conformance/CI.
- [train-deploy-unification.md](train-deploy-unification.md) — the loop unification
  (one engine, one step): Layer A/B/C, Phase 0/1/2, the qd + launch-IC divergences.
- [locomotion-shadowing-vs-pure-rl.md](locomotion-shadowing-vs-pure-rl.md) — why a
  kinematic ghost is not a balance solution, and the architecture choice for walking.
- [rl-two-layer-architecture.md](rl-two-layer-architecture.md) — the Ghost Method
  (Shadow + Ghost + Mimic) standard recipe.
- [g1-stand-rl-playbook.md](g1-stand-rl-playbook.md) — the early G1 stand journey,
  recipe, the 8 dead ends, and the floor-contact regression analysis (historical).
- [atlas-stand-rl-journey.md](atlas-stand-rl-journey.md) — porting to 30 DOF,
  the mass-DR lottery, the PPO ceiling.
- [omniquad-residual-rl.md](omniquad-residual-rl.md) — the quadruped model+residual
  recipe and the "policy is a passenger" finding.
- [h1-walk-rl-journey.md](h1-walk-rl-journey.md) — the H1 walk + deploy-physics
  fine-tuning journey: the run-3 champion (2.03 s fwd), why deploy-solver
  fine-tuning **regressed** the deploy, the `add_mjcf` matched-model fix, and the
  launch-IC + obs-pipeline lever toward a durable 10-20 m walk.
- [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md) — the generalized
  heavy-DR recipe (with the new scope caveat).
- [humanoid-balance-gap.md](humanoid-balance-gap.md) — historical "why bipeds
  are hard" analysis (its original LIPM conclusion was wrong; kept for context).
- [engine-migration-plan.md](engine-migration-plan.md) §13.3 — the P6/P8 Newton
  contact + statics work that the deploy gap is entangled with (this doc is the
  *accurate* low-level record of the 1.55 s state).
