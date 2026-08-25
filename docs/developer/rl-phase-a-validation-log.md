# RL Phase-A validation log — deterministic G1 balancer (Layer 1)

**Status: ✅ G1 STANDS in the deploy. Root cause found + fixed. Last session: 2026-06-10.**
The earlier "stands 5527 s" claim was RETRACTED (non-reproducible); the earlier
"spawn fold nudges CoM forward" diagnosis was *close but incomplete*. This session
found the actual root cause by stepping the deploy model in **plain mujoco** (no
Newton, no RL, just holding NOMINAL) and watching it tip forward at ~1.3 s — proving
it was **NOT a sim2sim gap and NOT the policy**. Two concrete causes, both fixed:

1. **Forward CoM at NOMINAL (the tip).** At the old pose (hip −0.20 / knee 0.42) the
   deploy model's whole-body CoM_x = **+0.005 m**, which is **5 mm AHEAD of the foot
   front (x = 0.0)** → it pitches forward and tips at ~1.3 s under *any* control
   (pure PD, analytic baseline, or RL — all verified). The OmniSim URDF importer
   places the foot ~35 mm further back than newton's native `add_urdf` (whose model
   CoM is 26 mm *behind* the foot front and stands at the shallow pose) — a genuine
   importer discrepancy worth a separate fix. **Workaround/fix:** a deeper-squat
   NOMINAL **hip −0.30 / knee 0.52** recentres the CoM behind the foot front →
   statically stable (holds 15 s in plain mujoco, pitch settles to +0.04, recovers
   from ±0.15 rad roll perturbations). Updated in both `g1_stand_deploy.py`
   (`NOMINAL_LEGS`) and `gpu_mjwarp_g1_stand_trainer.py` (`NOMINAL`).

2. **Analytic ankle PD destabilised roll.** Even at the stable pose the deploy fell
   in ROLL at ~0.9 s: the analytic ankle-roll PD's finite-difference `roll_rate`
   spikes at handover and kicks `ankle_roll`. **Fix:** the analytic ankle PD is now
   **OFF by default** (`_balance_gains` defaults 0; opt-in via `G1_BAL_*`). Active
   balance is the RL policy's job (residual on pure NOMINAL); the static pose itself
   needs zero control.

**Verified:** `run_g1_stand_deploy.ps1` (new NOMINAL + PD off + spawn-seed, on
mujoco_warp ke=400/kd=60) **STANDS** — roll ≈ 0, pitch steady +0.04, bz steady 0.776,
0 falls (10+ s and counting in a long headless run; a stable equilibrium, not a slow
tip). Confirmed independently in plain mujoco (mj_step) and mujoco_warp. The RL
residual policy (trained PD-off on the new NOMINAL with heavy DR) adds perturbation
rejection on top. **NOTE: §10 below ("SOLVE") and the old MARGINAL framing are
SUPERSEDED by this header.**

**Earlier verdict (2026-05-29): PENDING** — preserved below.

---

## Session 2026-06-09 (L5) — torque mode, a harness bug, and the stiffness finding

Ran on this machine: **RTX 5070 Ti Laptop GPU, 12 GiB** (note: the docs assume an
RTX 3060 — this box is faster, which helps reconcile the perf discrepancy flagged
in [rl-current-state.md](rl-current-state.md)). warp 1.13.0 / newton 1.2.0, from
PowerShell, no OmniSim rebuild (scripts step `newton` directly). New harness:
`_scratch/g1_balance_torque.py` (a local throwaway scratch probe — these
`_scratch/` balance probes were later untracked and removed from the tree).

### 1. `control.joint_f` WORKS under `SolverMuJoCo` (W3.1 was XPBD-specific)
The W3.1 finding ("`joint_f` is dead") is **solver-specific to XPBD**. Under the
deploy's MuJoCo solver, a +60 Nm constant torque on both ankle-pitch DOFs moves
the pelvis 0.08–0.17 m / 0.10–0.89 rad in 0.5 s vs the zero-torque trajectory —
i.e. `joint_f` maps to MuJoCo `qfrc_applied` and is **additive on top of the
position PD**. ⇒ a torque-mode law is reachable in the harness with no C++ change,
and the deploy-side `joint_f` write in `OmNewtonBackend` (an L1 file) is a
justified hand-off **that only needs to work under the MuJoCo solver, not XPBD**.

### 2. Position mode — full screen closed out (all fail)
The two previously-NOT-RUN configs are now run: `errx_damp` and `mix` **both fail**
(collapse like the rest). Final position screen: `passive 1.12s` is the best;
every active position config (`pitchPD_1.5/3/6`, `pitch_hip`, `errx_damp`, `mix`)
is ≤ passive. **Position-mode ankle is conclusively insufficient at ke=20.**

### 3. Torque mode (ankle, then ankle+hip) — also fails at ke=20
With the sign corrected (calibrated against §1: **+ankle torque pitches the trunk
forward**, so a forward fall needs **negative** ankle torque; the attitude PD
enters as `+kt_p·pitch`, the CoM/DCM term as `−kt_x·e_dcm`):
- **ankle-only** (kt_p=160, kt_pr=25, kt_x=300): the torque correctly ramps
  negative and **saturates at the −88 Nm effort cap by t≈0.21 s, yet still
  topples (0.66 s).** This is the **ankle-strategy CoP limit** — a flat foot can
  only transmit `m·g·(foot half-length)`, so commanding more ankle torque just
  tips the foot; it's not actuator-limited.
- **hip strategy** (best = `hip_pos`, kt_hp=120/kt_hpr=15): **0.90 s.** It *does*
  hold the trunk attitude (pitch stays ~−0.06 rad vs −0.55 ankle-only), but the
  **whole CoM translates off the feet** (dcm error 0.02→0.24, vx→0.51 m/s) while
  the **soft ke=20 legs sink** (z 0.78→0.63). Wrong-sign hip (`hip_neg`) is worse
  (0.18 s). This is textbook **capture-point-uncontrollable**: once the capture
  point leaves the foot you must *step* or fall.

⇒ No simple PD law (position or torque, ankle±hip) at ke=20 holds even ~1 s.

### 4. ⚠️ HARNESS BUG found + fixed — `eval_fk` does NOT reset `SolverMuJoCo` state
`newton.eval_fk(model, joint_q, joint_qd, state)` resets the **newton state
buffers** (body_q reads back at the nominal z=0.78) but **NOT the `SolverMuJoCo`
internal `mjData`** (velocities / warmstart / contacts). Reusing one solver to
screen many configs is therefore **only valid while no config falls hard**: after
a hard fall the reset is incomplete and every subsequent config inherits the
corrupted post-fall state — observed directly (each config started at the prior
config's *final* roll, then exploded on step 1 from a tiny torque). This
**invalidated the first torque/hip screens** (and means the prior position
screen's non-`passive` configs — `passive` ran first, clean — were likely
corrupted too; their "all fail" conclusion still holds, re-confirmed below with
the fix). **Fix:** rebuild a fresh model+solver per config (kernels are cached, so
it's cheap). **Relevance beyond this script:** the faithful trainer (method #2)
must rebuild/re-sync the solver on episode reset, not rely on `eval_fk` alone — or
every post-fall env reset trains on garbage.

### 5. 🔑 THE FINDING — a stiffer joint PD holds the passive stand; ke=20 is the root cause
With the harness fixed, a **passive NOMINAL hold (zero balance torque)** across
joint stiffness (kd=0.15·ke), fresh solver per config:

| joint PD | 8 s triage | 30 s gate |
|--|--|--|
| ke=20 (deploy default) | 1.12 s | — |
| ke=60 | 1.70 s | — |
| ke=150 | 1.87 s | — |
| ke=300 | — | **2.61 s — fail** |
| **ke=400 / kd=60** | **8.00 s** | **30.00 s — PASS ✅** |
| ke=500 | — | **2.10 s — fail** |
| ke=1000 | 1.73 s (overshoots, vx→2.0) | — |

**At ke=400, G1 stands passively for the full 30 s** (pitch 0.01–0.06 rad, z~0.776,
CoM error ~0.05 — rock-stable), clearing the Phase-A primary gate with **no
controller and no policy**. PD torque at the hold is ~20 Nm (well under the 88 Nm
cap — a legitimate hold, not clipped). **But the window is narrow + non-monotonic:**
ke=300 and ke=500 both diverge at ~2 s. Reading: too soft (≤300) → legs buckle /
CoM drifts off; sweet spot (~400) → holds; too stiff (≥500) → discrete-time PD
instability at the 62.5 Hz control rate. So stiffness is the single biggest lever
but **fragile alone** — a robust stand still wants active balance to widen the basin.

### Caveats (honest)
- **Legs-only model** (23 bodies, ~28 kg, no arms). The real deploy
  `g1_stand_deploy.omniworld` is the full G1 (~34 kg, arms raise the CoM → harder); the
  ke≈400 value and the window width **may shift** and must be re-tested there.
- ke=400's 30 s hold is undisturbed and fragile — it is **not** push-robust by
  itself (no active balance). That is exactly what Phase A asks (zero-policy
  *undisturbed* stand) and what Layer 2 (the RL residual) is meant to add.
- "nefc overflow — increase njmax" warnings appear during collapses (a
  contact-constraint cap); they're a symptom of the fall, not its cause (the
  pre-saturation trajectories are clean), but a clean post-fall might need njmax raised.

### 6. Deploy verification (real `g1_stand_deploy.omniworld`, full G1) — partial transfer, NOT 30 s
Ran the canonical [`g1_stand_deploy.omniworld`](../../projects/policies/research/worlds/g1_stand_deploy.omniworld)
**zero-policy** (`G1_BALANCE_FALLBACK=1`) through `headless_runner.py` on the
existing **06-08 binary** (Newton confirmed active in the sim log: `[OmNewtonBackend]
warp + newton imports OK`, all 13 hinge joints registered; NOT a silent ODE
fallback). `OMNISIM_NEWTON_TARGET_KE/KD` are read at runtime as **opt-in overrides**
([OmNewtonBackend.cpp:968](../../src/omnisim/physics/OmNewtonBackend.cpp)) — no
rebuild needed. Env: `STATICS=1 SUBSTEPS=4 FORCE_MUJOCO=1 MJWARP=1 URDF_USE_INERTIA=1 BASE_GUARD=1`.

| joint PD override | zero-policy survival | note |
|--|--|--|
| default URDF gains | **FALL@0.51 s** | fallen (bz=0.075) by t=1.0 |
| **ke=400 / kd=60** | **FALL@1.44 s** | **clean & level through t=1.0 (bz=0.783, roll/pitch≈0)** |
| ke=800 / kd=80 | FALL@0.51 s | too stiff → immediate instability (cf. harness ke=1000) |

**Verdict — the stiffness lever transfers PARTIALLY, but does NOT reproduce the
30 s harness hold on the full robot.** ke=400 ~triples the zero-policy stand
(0.51→1.44 s) and gives a genuinely level hold through ~1 s where default gains are
already face-down — same narrow non-monotonic window as the harness (ke=800 is too
stiff). **But the full G1 still falls at ~1.44 s ≈ the documented 1.55 s gap**,
where the legs-only harness held 30 s. The fall mode is the same capture-point CoM
translation (level briefly, then pitches forward) — **stiffness delays it, doesn't
fix it**. So the legs-only harness OVER-PROMISED by omitting the arms/torso (34 kg,
higher CoM). **Corrected conclusion: the ~1.55 s deploy gap is PART soft-PD (ke≈400
helps a lot early) and PART a genuine full-robot balance instability that stiffness
alone cannot close** — an active CoM/capture-point controller (the now-validated
`joint_f` torque path) and/or the RL residual is required for the full robot.
Caveats: 06-08 binary (misses today's L1 ball-joint/OmniQuad commits — both irrelevant
to G1 revolute standing); ke window not finely mapped on the full robot (400 best of
{default,400,800}); ankle effort_limit is 35 N·m on the real robot (88 in the harness).

### 7. Active capture-point controller + the harness↔deploy gap = a FORWARD DRIFT (L1)
Built an env-tunable position-mode capture-point/CoM balance law into the deploy
controller ([g1_stand_deploy.py](../../projects/policies/research/controllers/g1_stand_deploy/g1_stand_deploy.py)):
ankle attitude PD + CoM-velocity (DCM) term + hip-on-ankle-saturation + knee
height-hold, all gains `G1_BAL_*` env-overridable, **defaults == the legacy ankle
PD** (safe, opt-in). Then chased why ke=400 holds 30 s in the direct-Newton harness
but only ~1.5 s on the deploy — **same `g1_legs_omnisim.urdf`, same SolverMuJoCo,
same ke=400, same 16 ms/SUBSTEPS=4.** Ruled out, one at a time:
- **Ankle effort (35 vs 88 N·m): REJECTED.** Harness passive hold at ke=400 holds
  30 s with BOTH effort=88 and the deploy's real per-joint efforts (ankle=35) — at
  the stable NOMINAL the ankle torque is ~0, so the limit never binds.
- **The legacy analytic ankle PD: REJECTED.** Pure NOMINAL on the deploy (all
  `G1_BAL_*`=0, identical command to the harness) still **FALL@1.70 s**.

What actually differs: on the deploy the robot reaches a clean level stand (~1 s)
then **translates forward ~1 m** (bx 0.14→1.07) and topples — a forward drift the
direct-Newton harness does NOT exhibit at the identical NOMINAL command. Deploy
sweep (ke=400, zero policy):

| controller | survival | forward drift bx |
|--|--|--|
| pure NOMINAL | **1.70 s (best)** | →1.07 m |
| legacy ankle PD | 1.44 s | — |
| gentle active (kv_p=−1.0) | 1.31 s | →0.92 m |
| strong active (kv_p=−2.5, clamp .3, hip) | 1.06 s | →0.24 m |

**Ankle velocity feedback monotonically cuts the drift (1.07→0.24 m) but also cuts
survival** — at ke=400's high authority any ankle position activity over-rotates
(phase-lagged) and falls sooner. So **position-mode balance cannot rescue the
deploy**; the forward drift is the killer and it's absent in direct Newton with the
identical robot/solver/ke/command. **⇒ the harness↔deploy gap is an L1 deploy-pipeline
fidelity issue** (the OmniSim `motor.setPosition`→`OmNewtonBackend` actuation bridge,
and/or contact/friction/spawn), **not the controller** — same class as the known
g1-deploy actuation regression.

**Hand-offs to L1 (added):**
- **Deploy forward-drift / bridge fidelity:** a pure-NOMINAL zero-policy G1 holds
  30 s under *direct* `SolverMuJoCo` (the harness) but drifts forward ~1 m and falls
  at 1.7 s under the OmniSim deploy with the same robot/solver/ke/command. Repro:
  `g1_stand_deploy.omniworld` + `G1_BALANCE_FALLBACK=1 OMNISIM_NEWTON_TARGET_KE=400
  OMNISIM_NEWTON_TARGET_KD=60` + all `G1_BAL_*=0`. Closing this gap likely fixes
  the G1 deploy stand outright (no controller change needed).
- **(reiterated) joint_f deploy writer:** with the bridge gap closed, the harness's
  *torque*-mode capture-point law (which outperformed position mode) could deploy —
  but only once `OmNewtonBackend` exposes a `joint_f` write (live under MuJoCo).

**State of L5's deliverable:** the active capture-point controller is built, shipped,
and env-tunable (defaults safe); it demonstrably influences the deploy (drift
1.07→0.24 m). But a robust ≥30 s deploy stand is **blocked on the L1 forward-drift
bridge gap**, not on controller tuning — so L5 hands that off rather than burning
the slow (~90 s/run) deploy loop on gains that can't win against an engine quirk.

### 8. Spawn-pose ruled out + the Phase-A vs Phase-A′ reframing
- **`OMNISIM_NEWTON_SEED_POSE=1` (spawn in the squat, not straight-legged): REJECTED.**
  Deploy at ke=400 pure-NOMINAL + SEED_POSE=1 → **FALL@1.15 s** (worse: the pelvis
  sinks 0.78→0.68 and pitches forward harder). So the straight-leg→squat fold is not
  the cause either. The forward tip/drift is robust across **every** spawn pose, ke,
  and position-balance law tried → it is intrinsic to the deploy pipeline (L1), full stop.
- **Reframing (honest):** the Phase-A *gate as literally written* ("≥30 s zero-policy
  under SolverMuJoCo GPU mjwarp, SUBSTEPS=4") **is met** — the ke=400 NOMINAL hold in
  `build_g1_native` IS that solver and holds 30 s. So **G1 "stands" under the deploy
  *solver*.** What remains unmet is **Phase A′** ("the same hold in the real
  `g1_stand_deploy.omniworld`"), blocked by the L1 motor→Newton bridge forward-drift.
  Caveats keep this from being a clean "Phase A done": the ke window is narrow/fragile
  (ke300/500 fail) and the hold is not push-robust (Layer 2's job). Net: **the physics
  + the operating point for a G1 stand are proven; shipping it needs the L1 bridge fix,
  then basin-widening + the push residual.**
- **Basin-widening with active balance: does NOT work with PD-class laws.** The active
  capture-point torque law (kt_p=80/kt_pr=15/kt_x=120 + roll) at **ke=300** (where the
  passive hold collapses at 2.61 s) → **FELL@1.89 s** (no better than passive). Active
  position/torque PD balance does not convert the narrow ke≈400 knife-edge into a robust
  basin — consistent with the original finding (fixed-foot stand is capture-point-marginal;
  PD-class control insufficient). Robustness needs a WBC/capture-point-stepping controller
  (Phase E) or the RL residual (Layer 2), NOT more PD tuning. **The deterministic-control
  design space (position/torque, ankle/hip, stiffness sweep, active balance, spawn pose) is
  now EXHAUSTED across both harness and deploy.**

### 9. Regression guardrail — linked robots / Newton fidelity (no regression)
Ran the full Newton regression trio on the **2026-06-08 binary** (the deployed build;
today's L1 commits W2.2-ball-joint / OmniQuad-W6 are not in it — those are gated on push):
- **`newton_coverage` (robot corpus):** 8/8 worlds, **135/135 articulations Newton-eligible**, 0 mesh/joint gaps.
- **`physics_oracle --gate --require-newton`:** PASS (INV1 finite, INV2 settled, INV3 reproducible; Newton confirmed active, 0.275 m ODE-divergence).
- **`faithful_check` (robot corpus):** **5/8 FAITHFUL** (panda 0.000, rosbot 0.003, rosbot_xl 0.005, the two cobot-arm worlds 0.000) — **identical to the §8.1 baseline**; the 3 non-faithful are all documented non-regressions (ur_arms drift 0.062 m; omniquad W6 collapse; mavic = uncontrolled drone).
- **Verdict: NO regression** — Newton functions correctly on every linked robot that is meant to be faithful; the engine work this session (L5, pure `projects/policies/` Python + docs) touched no engine/other-robot files, so it cannot regress them. Current-main certification (today's L1 commits) is covered by the pre-push gate.

### 10. ✅ SOLVE — G1 stands indefinitely in the real deploy (friction was the last piece)
With full Newton access, chased the deploy forward-drift to its root. The harness uses
`mb.add_ground_plane()`; the deploy's ground is the RectangleArena floor as a STATICS
collider. Both use the same contact ke/kd (Newton default 2500/100 — verified equal) —
**but the difference was friction**: `OmNewtonBackend` defaults `OMNISIM_NEWTON_GROUND_MU=1.0`,
and its own in-file friction-probe comment ([OmNewtonBackend.cpp:128-135](../../src/omnisim/physics/OmNewtonBackend.cpp))
documents *"sphere feet SLIDE ~1 m while merely standing at mu=1.0 but PLANT (<4 cm) at
mu=2.0."* That is exactly the deploy's bx→1.07 m forward drift.

**Fix (no engine code change): `OMNISIM_NEWTON_GROUND_MU=2.0`** + the stiff PD (ke=400/kd=60).
Deploy result, zero policy (`G1_BALANCE_FALLBACK=1`):

| config | result |
|--|--|
| default mu=1.0 | tip / forward drift, ~1.7 s |
| **mu=2.0** | ⚠️ ~~STANDS 5527 s sim, 0 falls~~ **RETRACTED (non-reproducible)** — this was ONE run; on re-test the same config tips forward ~1.4 s on ~6/7 runs. The durable stand is the 2026-06-10 deeper-squat pure-pose fix (`f48f00b7`), not this μ=2.0 run. See [rl-current-state.md](rl-current-state.md). |
| mu=2.0 + ONNX policy | FALL@0.86 s (policy trained for soft ke=20; obsolete for the stiff config) |

⇒ **Phase A AND A′ COMPLETE: G1 ships standing with NO RL.** Deterministic Layer-1
(stiff position PD on NOMINAL + planted feet) is the product floor, exactly as the
two-layer architecture intended. Shipped via [`run_g1_stand_deploy.ps1`](../../projects/policies/research/runners/run_g1_stand_deploy.ps1)
(encodes the proven config); world header updated. Both fixes are launch config, so
engine defaults — and every other robot — are untouched (regression suite clean).
**Remaining (future, optional):** a Layer-2 push-recovery residual retrained on THIS
config (ke=400/mu=2.0) using the env-tunable `G1_BAL_*` law / the `joint_f` torque path;
and, as a robustness polish, the narrow ke window (300/500 fail passively) is moot for
the undisturbed stand but would matter under large pushes.

This is the running experiment log for **Phase A** of
[rl-two-layer-architecture.md](rl-two-layer-architecture.md): prove (or
disprove) that a **deterministic, hand-coded active balance controller** can
make G1 stand by itself — the load-bearing claim of the whole two-layer
architecture. No RL is involved in Phase A; the RL residual (Layer 2) only comes
after a deterministic controller holds the stand.

**Phase-A gate:** the controller holds, with ZERO policy, pelvis_z within ±5 cm
and |roll|,|pitch| < 0.15 rad for **≥ 30 s** under the EXACT deploy solver
(`SolverMuJoCo`, GPU mjwarp, SUBSTEPS=4). Passive (no controller) topples in
~1.1–1.5 s — that's the number to beat by ~20×.

---

## Environment / runbook (read first when resuming)

- **Run from PowerShell, not MSYS2 bash** (the embedded/standalone interpreter
  needs `warp` from user site-packages; a bash launch makes it invisible). ⚠ 2026-08-08:
  the consequence changed — it used to be a *silent ODE fallback*, and with ODE deleted
  (`bdc02139`) there is nothing to fall back to. Code-verified in
  `OmPhysicsBackendRegistry::newtonEnforced` / `OmNewtonBackend::refuseIfBrokenAndNewtonWanted`,
  the two cases differ: a runtime that is **installed but will not come up** is a FATAL
  (the engine refuses the world), whereas a runtime that is simply **not importable** —
  which is exactly what a bash launch produces — still falls through to the inert
  `OmOdeBackend` stub and runs with **no physics at all, silently**, unless you set
  `OMNISIM_REQUIRE_NEWTON=1` (then it fatals). **So set `OMNISIM_REQUIRE_NEWTON=1` on
  these runs** — the advice below is unchanged, but the failure it protects against is now
  "no physics" rather than "ODE physics", and it is no more visible than before. Verified
  toolchain: warp 1.13.0, newton 1.2.0, CUDA available (RTX 3060 Laptop, 6 GiB).
- Always use **`python -u`** (unbuffered) or output buffers and you see nothing
  until exit. Redirecting `> log 2>$null` hides the warp module-load spam.
- These scripts use `newton` directly — **no OmniSim binary rebuild needed.**
  They build the legs-only G1 as a native Newton articulation and step it with
  the exact deploy solver.

### Scripts

The `build_g1_native.py` foundation is tracked. The `_scratch/g1_balance_*`
probes below were local throwaway scratch (briefly force-added, then untracked
and removed once `/_scratch/` was gitignored) — they are **no longer in the
tree**; the descriptions are kept for the record only.

| Script | What it does |
|--|--|
| [`projects/policies/research/training/build_g1_native.py`](../../projects/policies/research/training/build_g1_native.py) | Builds the legs-only G1 native Newton model (POSITION_VELOCITY, ke=20/kd=3) + the exact deploy solver. The foundation everything here imports. |
| `_scratch/g1_balance_probe.py` *(removed)* | Passive NOMINAL hold, instrumented: logs pelvis_z, CoM, foot-midpoint, CoM-over-foot error, roll/pitch over time. Reproduces the ~1.5 s topple and shows *which way* it falls. |
| `_scratch/g1_balance_tune.py` *(removed)* | The deterministic balancer + a gain screen. `balance_offsets()` is a hand-coded PD feedback law (ankle/hip/knee position offsets on CoM-error / pelvis-pitch). Builds the solver ONCE and screens a `CONFIGS` list with a clean reset between trials. |

```powershell
# Passive baseline (the "before"):
python -u _scratch/g1_balance_probe.py

# Gain screen (edit the CONFIGS list in the file):
python -u _scratch/g1_balance_tune.py 2>$null

# Single config:
python -u _scratch/g1_balance_tune.py mycfg ka_p=3 ka_pr=0.6 sec=8 2>$null
```

### Model facts (from the probe)

- Legs-only native model: **23 bodies, total ~28 kg** (full G1 is 34 kg; this is
  legs+waist, arms absent). Actuated DOFs are indices **6–18** (`DOF0=6, NJ=13`);
  joint-q indices 7–19. Joint order = `g1_robot_spec.JOINT_NAMES[:13]`.
- 13-joint indices: hip_pitch 0/6, hip_roll 1/7, hip_yaw 2/8, knee 3/9,
  ankle_pitch 4/10, ankle_roll 5/11, waist_yaw 12.
- **Feet = bodies 7 and 13** (lowest z ≈ 0.037 at spawn). Pelvis = body 0.
- Ankle limits: pitch (−0.873, +0.524), roll (±0.262). NOMINAL ankle_pitch = −0.23.
- Control tick dt = 0.016 s (62.5 Hz), 4 substeps → physics at 250 Hz.
- CoM computed mass-weighted from `body_q` + `body_com`; **`getContactPoints`
  is not available** under Newton, so the support center is the geometric
  midpoint of the two foot bodies.
- **Solver reuse is valid:** `newton.eval_fk(model, model.joint_q, model.joint_qd, state)`
  on both state buffers cleanly resets to the NOMINAL start (the passive
  self-check reproduces the probe trajectory exactly). So one solver can screen
  many configs.

---

## Findings so far

### Passive failure mode (the target to fix)
The robot **pitches forward** (pitch 0 → −1.5 rad), CoM runs forward then the
body face-plants; pelvis_z collapses to ~0.06 (lying flat) by ~1.9 s. **The
instability is almost purely sagittal (pitch / x-axis); lateral (roll / y) is
stable.** So Phase A is mainly a sagittal ankle/hip pitch stabilizer.

### Gain screen results (position mode, 8 s target; passive ≈ 1.1 s)

| Config | Gains | Survived | Note |
|--|--|--|--|
| passive | — | **1.12 s** (fwd) | self-check, matches probe |
| ax_pos | ka_x=+6, ka_vx=+0.8 | 0.91 s (fwd) | +err_x ankle → tips forward *faster*; wrong sign |
| ax_neg | ka_x=−6, ka_vx=−0.8 | 1.25 s (**back**) | −err_x ankle rotates back but **overshoots** → falls backward |
| pitchPD_1.5 | ka_p=+1.5, ka_pr=+0.3 | **0.02 s** | instant forward collapse |
| pitchPD_3 | ka_p=+3, ka_pr=+0.6 | 0.02 s | instant forward collapse |
| pitchPD_6 | ka_p=+6, ka_pr=+1.2 | 0.02 s | instant forward collapse |
| pitch_hip | ka_p=+3, ka_pr=+0.6, kh_p=+1.5 | collapse | also hit `nefc overflow` (contact-solver cap) |
| errx_damp | ka_x=−3, ka_vx=−2.5 | NOT RUN | (killed before reaching) |
| mix | ka_p=2, ka_pr=0.4, ka_x=−1.5, ka_vx=−1.5 | NOT RUN | (killed before reaching) |

**Interpretation:** no position-mode gain set tried has held the stand — they
either fall as fast as passive or *faster* (overshoot/oscillation). This matches
the balance-control survey's prediction: a position-target offset fed through a
soft PD (ke=20) is a low-authority, phase-lagged way to do an ankle strategy and
does not cleanly control the LIP's one unstable mode (the DCM). The pitch-PD
derivative term on a noisy 62 Hz finite-difference appears to destabilize
immediately. **This is suggestive, NOT a disproof** — position mode is not yet
exhausted (see resume list).

### Sign convention learned
A **forward** fall is pitch < 0 / err_x > 0. A **negative** ankle-pitch offset
(plantarflex) rotates the body **back** (confirmed by ax_neg). So err_x feedback
gains are negative; pitch feedback needs the matching sign (the positive ka_p I
tried collapsed it — revisit the sign/derivative-filtering on resume).

---

## Architecture verdict so far: PENDING (2026-05-29 — superseded by the 2026-06-09 session above)

- **Layer 1 (deterministic stand): NOT yet achieved.** Naive position-mode
  ankle/pitch feedback does not hold. Position mode is looking marginal but is
  not fully tested.
- **Not disproved:** the decisive experiment — **torque mode** — has not been
  run. The user pre-approved adding a torque (`joint_f`) path.
- **Layer 2 (residual beats baseline under push): not started** — independent,
  cheap to validate on OmniQuad.

## Architecture verdict (2026-06-09 update)

- **Layer 1 (deterministic undisturbed stand): ACHIEVED in the harness, via joint
  stiffness, not via a balance controller.** A passive NOMINAL hold at ke=400/kd=60
  clears the 30 s gate (zero policy). Naive PD balance laws (position OR torque,
  ankle±hip) at the deploy's ke=20 all fail (<1 s) — but that was the wrong knob;
  **the joint PD stiffness was the dominant variable all along.**
- **Caveat — fragile + unproven on the real deploy.** The ke window is narrow
  (300/500 fail) and the result is legs-only; the full-robot `g1_stand_deploy.omniworld`
  test is the gating next experiment before claiming Phase A′.
- **Active balance is now a robustness/basin-widening tool, not a prerequisite for
  the bare stand** — and it should be tuned on the stiff operating point (sign now
  calibrated; the torque law in `g1_balance_torque.py` is ready to layer on).
- **Layer 2 (residual beats baseline under push): not started** — still the
  independent, cheap-on-OmniQuad validation; now has a standing G1 base to sit on.

---

## Resume here (ordered next steps — revised 2026-06-09)

1. **[PRIME, L1/deploy hand-off] Test joint stiffness on the REAL deploy.** Set
   `OMNISIM_NEWTON_TARGET_KE` (deploy default 20) toward ~400 (with matching kd)
   and run the canonical [`g1_stand_deploy.omniworld`](../../projects/policies/research/worlds/g1_stand_deploy.omniworld)
   with **zero policy** — does the full G1 (34 kg, arms) stand ≥30 s? No rebuild
   needed (env var). This is the experiment that turns the harness finding into a
   shipped Phase-A′ ("G1 stands with no RL"). If it stands, the long-standing
   1.55 s deploy gap was a soft-PD default, not a controls problem.
2. **Map the robust ke window on the full robot** (it will differ from the
   legs-only ~400; CoM is higher). If the window is as narrow there as here,
   **layer the (now sign-correct) active torque balance** from
   `g1_balance_torque.py` onto the stiff operating point to widen the basin —
   that is where active balance earns its keep, not on the bare stand.
3. **Promote to the shared module** `projects/policies/control/g1_balance.py` (per the
   architecture interface contract) once a robust stand holds: the stiff-PD config
   + any active trim, imported by BOTH `build_g1_native` and the deploy controller
   (single source of truth — kills the train↔deploy baseline-drift bug).
4. **Then Layer 2:** OmniQuad push-recovery A/B
   ([`eval_push_recovery.py`](../../projects/policies/research/tools/eval_push_recovery.py),
   `--policy` vs `--no-policy`, matched seeds) to settle the never-measured
   passenger-on-push question; then the G1 residual on the stiff standing base.

**Hand-offs to L1 (Newton core)** — left here per lane discipline, NOT edited into
`OmNewtonBackend` by L5:
- **`OMNISIM_NEWTON_TARGET_KE=20` is too soft for a humanoid stand.** ~400 holds
  in-harness. Consider a per-robot / higher humanoid default (and verify the 88 Nm
  effort cap isn't the binding constraint at the higher gain).
- **`control.joint_f` is live under `SolverMuJoCo`** (maps to `qfrc_applied`),
  dead only under XPBD (W3.1) — so a deploy-side `joint_f` writer is viable for the
  MuJoCo path, scoping W3 down to the solver that actually needs it.
- **`eval_fk` does not reset `SolverMuJoCo` `mjData`** — relevant to any engine /
  trainer reset path that assumes it does.

## See also
- [rl-two-layer-architecture.md](rl-two-layer-architecture.md) — the architecture spec.
- [rl-current-state.md](rl-current-state.md) — canonical cross-robot status.
- [g1-stand-rl-playbook.md](g1-stand-rl-playbook.md) — the G1 journey + the 1.55 s deploy gap.
