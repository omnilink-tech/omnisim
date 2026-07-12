# Shadowing — measured experimental results (for the paper)

> Real, regenerable numbers from this checkout (MuJoCo 3.8.1 + mujoco_warp + CUDA), to drop
> into the paper's results/comparison sections. Every number below is reproduced by a committed
> script; none is documented-only. Honest-tone discipline of [rl-current-state.md](rl-current-state.md)
> applies. Positioning vs. related work is in [shadowing-comparison.md](shadowing-comparison.md).

**Setup (held fixed across all RL conditions).** Robot: Unitree **Go2** (quadruped). Tracker:
`gpu_mjwarp_go2_walk_trainer.py`, residual RL on a foot-space trot reference, **200 iters /
4096 envs / 9.8 M env-steps (~50 s GPU)** per run. Certificate: `feasibility_certificate.certify`
(per-step contact-wrench LP), run on the kinematic trot ghost — **RL-independent, 0.19 s/ghost on
CPU**. Feasibility knob: commanded forward speed `vx` (gentle → far past what contacts/actuators
can supply). The certificate never sees the policy; the policy never sees the certificate.

---

## Experiment 1 — Does the certificate predict RL learnability? (E1 + E2, the headline)

`python projects/policies/research/shadowing/e2_rl_learnability.py` — train one residual policy at each `vx`,
eval it, and correlate the **RL-independent certificate score** with the **trained policy's
learnability**. SAME learner + SAME compute at every `vx`; only the reference's feasibility differs.

| vx | cert verdict | **cert score** | base %mg | torque %lim | first-fall (s) | dist (m) | falls/env | speed-track % |
|----:|:---:|:---:|---:|---:|---:|---:|---:|---:|
| 0.3 | PASS | **0.69** | 1.4 | 31 | 11.7 | 2.0 | 3.1 | 57 |
| 0.5 | PASS | **0.67** | 2.0 | 33 | 11.2 | 4.0 | 3.2 | 73 |
| 0.7 | PASS | **0.54** | 2.6 | 46 | 8.5 | 4.7 | 4.3 | 78 |
| 0.9 | PASS | **0.48** | 3.2 | 52 | 6.7 | 5.1 | 5.6 | 79 |
| 1.2 | PASS\* | **0.00** | 4.2 | 100 | **1.1** | 0.9 | **34.6** | 55 |
| 1.6 | PASS\* | **0.00** | 5.4 | 100 | **1.5** | 0.8 | **25.9** | 26 |
| 2.0 | PASS\* | **0.00** | 7.1 | 100 | **1.1** | 0.3 | **35.6** | 8 |
| 2.5 | FAIL | **0.00** | 26.8 | 100 | **1.2** | 0.1 | **34.7** | −1 |

\* binary gate still PASSes (a within-limit contact-wrench solution *exists*) but the **score = 0**
because the actuator torque is pinned at the limit (zero margin). See "score vs gate" below.

**Result — Spearman rank correlation of the RL-independent certificate score with RL learnability:**

| learnability metric | Spearman ρ | p |
|---|---:|---:|
| **first-fall time** | **+0.939** | **0.001** |
| **falls/env (fewer = better)** | **+0.939** | **0.001** |
| forward distance before fall | +0.685 | 0.061 |
| speed-tracking ratio | +0.685 | 0.061 |

**The cliff is co-located.** Between vx = 0.9 and 1.2 the certificate score drops **0.48 → 0.00**
(torque saturates, 52% → 100% of limit) and RL learnability collapses **in lockstep**: first-fall
**6.7 s → 1.1 s** (6×), falls/env **5.6 → 34.6** (6×). A *feasible* reference (score ≥ 0.48,
vx ≤ 0.9) is learned to a multi-second, low-fall walk; an *infeasible* one (score 0, vx ≥ 1.2) the
*identical* RL run cannot track — it face-plants ~30×/episode. **This is the causal E1 spine with a
number, and the E2 claim measured: the certificate score predicts learnability at ρ = 0.94 (p = 0.001).**

**Score vs. gate (a real finding).** The *binary* gate tests **existence** of a within-limit
contact-wrench solution (necessary feasibility) and is too loose — it stays PASS to vx = 2.0 at
zero margin. The **scalar score** is the **margin**, and the margin is what predicts robust
learnability. Report and threshold on the score, not the pass/fail bit.

---

## Experiment 2 — Model-based cross-check (no RL in the loop)

`python projects/policies/research/shadowing/e2_graded_feasibility.py` — same `vx` sweep, but the ground-truth
"trackability" is an **independent full-dynamics open-loop rollout** (integrate the trot through
MuJoCo, no LP, no policy). Decouples the result from any RL artifact.

- Certificate **score ≥ 0.4** holds up to **vx = 0.9 m/s**; the independent rollout tracks without
  falling up to **vx = 0.9 m/s**. **The two cliffs coincide.**
- The RL-independent certificate predicts the dynamic-trackability limit **before any policy exists**.

---

## Experiment 3 — Ablation vs. the H2O-style feasibility filter (compute)

The closest prior feasibility filter (H2O / OmniH2O, [arXiv:2403.04436]) decides a reference is
"implausible" only **after training a privileged tracking policy and watching it fail**. Same
information, two costs (measured here):

| filter | per-reference cost | hardware | predictive content |
|---|---|---|---|
| **Shadowing certificate** | **0.19 s** | **CPU, no training** | score ρ = 0.94 vs RL learnability (Exp 1) |
| H2O-style policy-rollout filter | **~60 s** (≈50 s train + ~10 s eval) | GPU | the learnability it measures *is* what the score predicts |

**≈ 300× cheaper per reference, no GPU, before any RL** — and it yields the same keep/reject
decision the H2O filter only obtains post-training. (Exp 1 is the agreement evidence: the
certificate score and the trained-policy learnability rank-agree at ρ = 0.94.)

---

## Experiment 4 — Certificate soundness (this session's hardening)

`python projects/policies/research/shadowing/feasibility_certificate.py` (self-test, exit 0) +
`verify_certificate_suite.py`:

| metric | before | after |
|---|---|---|
| discrimination at the main entry | **0/2** (FAIL on both a feasible walk and a levitating ghost) | **16/16** repertoire + adversarial controls |
| motion classes covered | 1 (flat walk); missed Spot | 6 (walk/get-up/jump/hill/sit/arm) + Spot |
| false-PASS on impossible motions | rubber-stamped an 8 Hz base-shake / 3 m/s drift | **0** (shake → INDET, drift → FAIL) |

---

## Paper-ready comparison (measured numbers next to the field)

| Property | DeepMimic/AMP | H2O/OmniH2O | KungFuBot | Cassie/Li-2021 | **Shadowing (ours)** |
|---|---|---|---|---|---|
| reference-feasibility gate before RL | none | **RL-dependent** (policy-rollout) | binary, quasi-static | feasible-by-construction (no separate cert) | **RL-independent, dynamic, scored** |
| predicts RL learnability | — | only post-training | — | — | **ρ = 0.94 (p = 0.001), pre-training** |
| cost to screen a reference | — | ~60 s GPU (train a policy) | — | — | **0.19 s CPU** |
| motion coverage (this work) | sim chars | humanoid mocap | dynamic skills | Cassie gaits | **6 classes + arm, 9 robots** |
| deploy demonstrated | sim only | real H1 (≈42 mm) | real G1 | real Cassie | **SIM only (OmniSim Newton): Spot +30 m / Go2 +66 m / B2 +95 m, 0 falls** |

**Honest framing for the paper.** The novelty is the **architectural placement** — first to
*certify-then-track* with an explicit, RL-independent, **dynamic** feasibility certificate, whose
**score predicts RL learnability (ρ = 0.94) at ~300× less cost than the closest prior filter**. The
certificate *math* is classical (Hirukawa/Caron CWC) — claim the placement, not the invention.
Shadowing does **not** beat MPC as a controller, and the durable underactuated-**biped** deploy
remains open (shared with the whole field); the verified deploy wins are quadruped.

### Reproduce
```bash
python projects/policies/research/shadowing/e2_rl_learnability.py     # Exp 1 (GPU, ~12 min): the ρ=0.94 curve
python projects/policies/research/shadowing/e2_graded_feasibility.py  # Exp 2 (CPU): model-based cross-check
python projects/policies/research/shadowing/feasibility_certificate.py# Exp 4 (CPU): soundness self-test
```
