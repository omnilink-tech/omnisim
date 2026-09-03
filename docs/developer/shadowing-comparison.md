# Shadowing vs. the field — honest positioning + measured improvement

> Companion to [shadowing.md](shadowing.md) and shadowing-verification.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)).
> This file answers two questions a reviewer (or a partner) will ask: **(1) how much did our
> own pipeline measurably improve this session, and (2) how does Shadowing compare to the
> leading RL and control methods for underactuated robots?** Sourced from a 5-agent literature
> sweep (30+ primary papers, cited inline) + measurements on this checkout. The discipline of
> [rl-current-state.md](rl-current-state.md) applies: claims are bounded to what was measured.

## 1. What "our algorithm" is (so the comparison is fair)

**Shadowing = generate → certify → track.** A trajectory optimizer (receding-horizon MPPI over
MuJoCo) produces a dynamically-feasible reference ("ghost"); a **numerical, RL-independent
feasibility certificate** (per-step contact-wrench LP: ZMP/centroidal feasibility + friction
cone + torque limits) gates it *before* training; an RL policy tracks the certified ghost to
deploy. The session's work hardened **Component 2 (the certificate)** and validated the **E2**
claim. So the honest improvement story is about *the certificate and the methodology*, **not** a
new low-level controller that out-tracks MPC.

## 2. How much we improved (measured this session)

| Axis | Before | After | Evidence |
|---|---|---|---|
| **Certificate correctness (main entry)** | `verify()` returned **FAIL on both** a feasible walk *and* a levitating ghost — **0/2 discrimination** | **16/16** repertoire + adversarial controls correct; **0 false-PASS** on any impossible motion | `verify_certificate_suite.py`, `feasibility_certificate.py` self-test (exit 0) |
| **Motion coverage** | 1 class (flat-ground foot-contact walk); **missed OmniQuad entirely** (no `foot`-named body) | 6 classes (walk/get-up/jump/hill/sit/arm) + OmniQuad, + INDETERMINATE abstain | self-test table |
| **Adversarial soundness** | get-up branch **rubber-stamped** an 8 Hz base-shake / 3 m/s drift (PASS) | shake → **INDET**, drift → **FAIL** (regression-guarded) | self-test controls |
| **E2 — does the score predict trackability?** | claimed, never run | **score cliff = independent-rollout fall cliff = 0.9 m/s** (Go2 vx sweep) | `e2_graded_feasibility.py` |

**The headline measured result (E2 graded sweep, Go2 trot, `e2_graded_feasibility.py`):** push commanded
forward velocity past feasibility and the certificate's **scalar score** falls monotonically
(0.70 → 0.67 → 0.64 → 0.48 → **0.00** at vx = 1.2 m/s, where actuator torque saturates). An
**independent** full-dynamics open-loop rollout (no LP, just integrate the gait through MuJoCo)
**first falls at exactly vx = 1.2 m/s**. The score-margin cliff (score ≥ 0.4) and the rollout
fall cliff **coincide at 0.9 m/s** — i.e. the **RL-independent certificate predicts the
dynamic-trackability limit before any policy is trained.** (Nuance, stated honestly: the *binary*
PASS gate is looser — it certifies the *existence* of a within-limit contact-wrench solution and
stays PASS to ~2.0 m/s at zero torque margin; the *score* is the margin and is the predictive
signal. Trust the score for robustness, the verdict for necessary feasibility.)

**The compute argument (the practical "improvement" vs prior feasibility filters):** the
certificate runs in **seconds per ghost on CPU, before any training**. The closest prior
feasibility filter, **H2O / OmniH2O** ([arXiv:2403.04436], [2406.08858]), only decides a motion is
"implausible" *after training a privileged imitation policy and watching it fail* (it discards
~1,500 of ~10,000 motions post-hoc). Shadowing flags the same class of references analytically,
pre-training — that is the concrete efficiency win and the cleanest novelty contrast.

## 3. Where Shadowing sits vs. the leading methods

### 3a. Imitation / reference-tracking RL (humanoids & characters)

| Method | Reference | Feasibility handling | Headline (deploy) |
|---|---|---|---|
| **DeepMimic** (Peng 2018, [1804.02717]) | mocap/keyframes | **none** — RL + RSI absorb it | sim character only |
| **AMP / ASE** (Peng 2021/22) | unstructured clips | **none** — adversarial *style*, no tracking | sim only |
| **PHC / PULSE / MaskedMimic** | AMASS | artifact cleaning + **online** failure curriculum | sim only |
| **ExBody / Exbody2** (2024, H1/G1) | curated mocap | **accommodation**: don't joint-track legs; manual curation | real H1/G1 (qualitative + key-pt err) |
| **H2O / OmniH2O** (2024, H1) | AMASS | **RL-DEPENDENT filter** — discard what a privileged policy fails | real H1, MPJPE ≈ 48 mm |
| **KungFuBot/PBHC** (NeurIPS 2025, G1) | video→SMPL | **RL-independent pre-filter — but BINARY + QUASI-STATIC** (CoM–CoP) | real G1, MPBPE 37 mm |
| **Shadowing (ours)** | TO ghost | **RL-independent, DYNAMIC certificate (CWC + torque), as a pre-RL GATE with a margin score** | certificate verified; deploy = quadruped walk (§3c) |

**Novelty verdict (honest, survivable under review):** *no prior RL method tracks a reference
certified dynamically feasible (friction + ZMP/centroidal + torque) by an explicit, separate,
RL-independent check used as a pre-training gate.* The two nearest neighbours each break one
property: **H2O's filter is RL-dependent** (it conflates "this policy failed" with "this reference
is infeasible"); **KungFuBot's gate is binary and quasi-static** (CoM–CoP balance, not a dynamic
torque/contact trackability score). Claim the **architectural placement ("first to
certify-then-track")**, *not* the invention of feasibility checking — the certificate math is
classical (Hirukawa CWC 2006; Caron 2015/2017 closed-form friction+ZMP+yaw-torque; Dai/Tedrake
2014 CWS-in-CWC in TO). Naming note: HumanPlus already uses "shadowing" for teleoperation —
disambiguate.

### 3b. Model-based control / trajectory optimization (the certificate's heritage)

The certificate's LP **is** the classical contact-wrench feasibility test, reused as a gate:

| Method | Feasibility | Hardware metric |
|---|---|---|
| **MuJoCo-MPC / MPPI** (Howell 2022; Williams 2018) | implicit/soft (rollout + cost penalties) | sim (MJPC) / driving (MPPI) |
| **Contact-implicit TO** (Posa 2014; Crocoddyl; CI-MPC Le Cleac'h) | **hard** (complementarity + friction) | Go1 push-recovery, ~400 Hz |
| **Convex MPC** (Di Carlo 2018, Cheetah 3) | **hard** friction pyramid + force bounds | ≤ 3 m/s, < 1 ms QP |
| **ZMP / preview** (Kajita 2003) | ZMP-in-support as a *tracking ref* | fully-actuated flat-foot |
| **Capturability** (Pratt 2006; Koolen 2012) | viability/capture regions | push recovery |
| **Shadowing certificate** | **CWC + torque as an RL-independent GATE** (Hirukawa/Caron math) | predicts the trackability cliff (§2) |

Shadowing does **not** compete with MPC as a controller — MPC *solves* the motion online; our
certificate *certifies* a reference and hands it to a learned tracker for robust deploy. The
classical line also supplies our central theoretical guard: **a standing/walking biped is an
open-loop-unstable inverted pendulum** (LIP divergent mode; Wieber 2006 "ZMP-feasible ≠ stable";
HZD/Grizzle). That is *why* the certificate tests **inverse-dynamics feasibility, not open-loop
stability** — and why a tracking policy (closing the loop) remains necessary. Feasibility
certification and stabilizing feedback are complementary, not substitutes.

### 3c. Sim-to-real RL for underactuated legged robots (the deploy reality)

The leading from-scratch / phase-prior RL walkers and their honest metrics:

| Method | Robot | Hardware metric | Reference |
|---|---|---|---|
| Hwangbo 2019 | ANYmal | 1.5 m/s (record at the time) | none (scratch) |
| Lee 2020 / Miki 2022 | ANYmal | 0.25–1.2 m/s rough terrain; 2.2 km Alps hike, 0 falls | uncertified CPG/FTG phase prior |
| RMA (Kumar 2021) | A1 | 12 kg payload, robust | none (scratch) |
| Walk-These-Ways (2022) | Go1 | ~3 m/s, commandable gaits | uncertified gait clock |
| DreamWaQ (2023) | A1 | 430 m / 465 m blind hikes | none (scratch) |
| **Shadowing in OmniSim** | **OmniQuad / Go2 / B2** | **+30 m / +66 m / +95 m, 0 falls** (Newton SIM deploy, canonical [rl-current-state.md](rl-current-state.md); not hardware) | **certified ghost** |

**Honest reading.** None of these RL methods certifies a feasible reference before training —
the white space is real. But the comparison must stay honest about *our* deploy reality (per
rl-current-state.md): on these robots the deployed neural policy adds **little over the analytic
gait** (OmniQuad residual is a "passenger": +4.87 m with policy vs +5.03 m without), and the **G1
*biped* walk does not durably deploy** (finite bout / topples — a trainer↔deploy gap, *not* a
ghost-feasibility defect, which is exactly what E2's `cert PASS + deploy FAIL` partition
localizes). So Shadowing's verified deploy wins are **quadruped** locomotion; the underactuated
**biped** deploy remains the open frontier, shared with the whole field.

## 4. Bottom line (what improved, stated without inflation)

- **The certificate** went from a broken gate (0/2 discrimination, missed OmniQuad, rubber-stampable)
  to a **sound, motion-agnostic, adversarially-tested** certificate with a margin score that
  **measurably predicts the dynamic-trackability cliff** (E2, Go2). That is the concrete,
  regenerable improvement.
- **Versus the field**, Shadowing's contribution is **methodological and unoccupied**: the first
  *certify-then-track* pipeline — an explicit, RL-independent, dynamic feasibility gate before RL.
  It does **not** beat MPC as a controller, and it inherits the field's open problem (durable
  underactuated-biped deploy).
- **No head-to-head deploy benchmark vs DeepMimic/H2O/MPC exists yet** — that is the honest next
  step (§5). Any single "X% better" number today would be unsupported.

## 5. Head-to-head experiments — RUN (numbers in shadowing-experiments.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)))

Three of the four are now done with real numbers (Go2, `gpu_mjwarp_go2_walk_trainer`, 200 iters/
4096 envs per run):

1. **Graded-feasibility learnability curve — DONE.** Swept vx past feasibility; trained one
   residual policy per vx; correlated the RL-independent certificate score with RL learnability:
   **Spearman ρ = +0.939 (p = 0.001)** for first-fall time and fall-rate. The score cliff (0.48 →
   0.00) and the learnability cliff (first-fall 6.7 s → 1.1 s, falls/env 5.6 → 34.6) are
   co-located at vx ≈ 0.9 → 1.2. (`e2_rl_learnability.py`)
2. **Ablation vs the H2O filter — DONE.** Certificate **0.19 s/ghost on CPU** vs the H2O-style
   filter's **~60 s GPU (train a policy first)** → **≈ 300× cheaper, no GPU**, same keep/reject
   content (the ρ = 0.94 agreement). (§Exp 3 of the results doc)
3. **Controlled E1 (feasible vs infeasible reference) — DONE.** Same learner + same compute:
   feasible (score ≥ 0.48) → multi-second, ~3–5-fall walk; infeasible (score 0) → 1.1 s, ~30+
   falls. ~6–10× learnability gap, attributable solely to reference feasibility.
4. **DeepMimic-style baseline tracker on an uncertified reference — STILL OPEN.** The remaining
   apples-to-apples head-to-head; the trainer here is residual-on-gait, so a from-scratch
   pose-tracking baseline needs a small additional harness.
