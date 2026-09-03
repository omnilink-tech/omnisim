# G1 — "Walk like the ghost ≥80%" : the full journey (honest journal)

> 📍 **Canonical G1 status:** [rl-current-state.md](rl-current-state.md) (the *G1 — detail*
> section) is the single source of truth. If a status claim here disagrees with that file,
> that file is right — fix this one.

**Date:** 2026-06-18.
**Goal (user):** *"Calculate numerically the percentage of how close G1's walk is to the
ghost, and keep going until G1 walks at least 80% similar to the ghost."* No eye-witness —
numbers only.

**One-line outcome:** We built the numerical metric, found the ambitious human ghost is a
**physical wall (~67%)**, and a **feasible ghost** lifts the *shape* match to **84–88%** — **but
that number is over a ~7 s walk window; the policy topples ~7 s, and the OmniSim deploy falls
sooner.** So the ghost-*similarity* goal is met **over the walk window**; a **durable,
indefinitely-walking ≥80% deploy is NOT achieved** — it is entangled with the unsolved,
months-long **G1 trainer↔deploy durability gap**. This doc records exactly what was tried, what
worked, what didn't, and why — so the next person starts from the truth.

> **Read this caveat first.** An early claim in this session ("84–88%, 128/128 upright →
> solved") was measured over a **3 s** eval window. A later **18 s** eval showed max upright
> survival **~6–8 s**. The 128/128 was a short-window artifact. The lesson is baked into the
> tooling now (always pair the % with a long-horizon survival check).

---

## 1. How we measure "% similar to the ghost" (the metric)

`gpu_newton_g1_walk_trainer.py --eval-ghost-similarity <policy.pt>` rolls the **greedy** policy
in the **DR-off** Newton parity env and, over a steady window (skip the ramp), computes per joint:

- **Amplitude fidelity** `max(0, 1 − RMSE/PTP_fair)` where `PTP_fair = max(ghost PTP, 0.35)` (the
  0.35-rad floor stops a near-static joint from reading as 0%).
- **Shape** = `max(0, Pearson r)` between achieved and ghost trajectories.

Headlines: **FAIR all-13**, **hips+knees+waist (9)**, **sagittal (6)**, **moving-joints**
(non-static), plus a **SHAPE %**. It has an identical-input **self-test** (achieved==ghost must
score ~100%) that gates every run. **It is honest about WHAT it measures, but it only measures
the window you give it** — and `upright envs used N/128` + `mean steady-window steps/env` must be
read alongside the % (a high % over a 3 s window says nothing about second 8).

Commits: `8bc5c2e9` (metric), `148e37ae` (fair + self-tested), `d7ffdd8c` (`--eval-every` live
in-training readout).

---

## 2. The ambitious human ghost — a physical wall at ~67%

Tracking the wide human/winter ghost (`lateral=lipm, yaw=human, lat_hip_amp 0.35`):

| policy | FAIR all-13 | moving | upright (3 s win) |
|---|---|---|---|
| early (comfix) | 44.6 % | — | — |
| W4 (`ghostwideFF`, full reward stack, ke150) | **63 %** | 67 % | 54–65 / 128 |
| S1 (`ghostSAG`, + `--track-sagittal-w 2.0`) | **66.7 %** | 68.6 % | 54 / 128 |

**Why it stalls (the key physics):** `similarity = 1 − RMSE/PTP`. A balancing biped **must**
deviate ~0.17 rad from a *kinematic* reference to stay up — that deviation **is** the residual
that keeps it upright. So RMSE is floored → similarity caps ~70 %, and ~half the robots fall
within the window. The gap is **structural, not optimization.**

### Levers tried against the human ghost (all logged, most negative)

| lever | flag | result |
|---|---|---|
| stiffer joint gains | `keNNN` (ke150→200) | reward ↑, similarity **unchanged** (64% ≈ 63%). Gains axis exhausted. |
| reward re-tuning | rw_track / sigma | W5 (softer) **worse**. W4 is the tuning peak. |
| **L/R mirror-symmetry loss** | `--mirror-loss` | **HURT** — W=1.0 collapsed the gait to a trivial symmetric stand; W=0.5 destroyed the sagittal shape (r 0.5→0.01). Kept in code, **default off**. (`d693e4c1`) |
| **sagittal swing up-weight** | `--track-sagittal-w` | **helped** — hip_pitch shape r 0.47→0.67, ankle 0.42→0.56, all-13 63→66.7%, but traded stability (more swing ⇄ more falls). (`72b98f87`) |
| height-penalty relax | `--height -10→-6` | bigger hip swing (the height penalty was paying the policy to keep hips stiff to avoid CoM bob), but **17/128 upright** — too unstable. |

Root cause of the *damped* hip swing (found by a parallel agent, code-grounded): the `height=-10
@ z_ref=0.74` penalty punishes the CoM bob a real hip swing produces, so the policy parks the
hips stiff. It's an **incentive** gap, not authority (the knee already tracks a larger swing).

**Insight that ended the human-ghost path:** *shrinking* the ghost amplitude makes the score
**worse** (same RMSE ÷ smaller PTP). The only way to raise the % is to lower RMSE — i.e. give the
robot a ghost it **doesn't need to deviate from** = a **dynamically feasible** reference.

---

## 3. The feasible ("achieved") ghost — shape match 67% → 88%

The `(B) achieved` shadow already existed (`b4835405`, [g1-improved-shadow.md](g1-improved-shadow.md)):
the robot's **own** gait, phase-binned + **L/R-symmetrised** (half-cycle mirror) + smoothed →
"feasible by construction, tracking floor ≈ 0." It was built in **mjwarp**, but the deploy is
**Newton** — so we added **`--build-achieved`** (`cae390b7`) to extract it **in the Newton deploy
solver** from the stable champion `ft_pdoff_clamp` → `datasets/g1_achieved_gait.npz`
(`8d447e32`); `00e20c5c` allowed `--gait-style achieved`.

A fresh policy trained to track it (`gpu_newton_g1_walk_ACH2_pdoff`, `--gait-style achieved
--rw-track 8 --track-sigma 0.03 --track-ankle-w 1.0 --iters 4000`), measured vs the ghost
**over a 3 s window**:

| metric | value (3 s window) |
|---|---|
| FAIR all-13 | **84.2 %** |
| hips+knees+waist | **86.1 %** |
| sagittal swing | **87.0 %** |
| moving joints | **87.9 %** |
| upright (3 s) | 128 / 128 |

Per-joint 0.72–0.90; shapes knee r 0.89–0.94, hip_pitch 0.88. The tracking error collapsed
(RMSE ~0.04–0.10 vs ~0.17 against the human ghost) — **the feasible-reference idea is correct.**

---

## 4. The correction — it topples ~7 s (durability ≠ similarity)

An **18 s** eval (`--eval-steps 1500 --eval-window-s 18`):

> `no env stayed upright for ≥60% of the steady window — max upright-window steps over envs was
> 375` → **~6 s** (≈8 s with the 2 s ramp).

So ACH2 walks ~7 s matching the ghost ≥80%, then **falls**. The 3 s "128/128" was a short window.

**Root cause — the trainer↔deploy durability gap, compounded:**
- The achieved ghost was extracted from the champion's gait **in the trainer's Newton env, where
  the champion survives only ~7.3 s** — *the same champion that walks 33.8 s in the `omnisim-bin`
  deploy.* The reference (and the policy that tracks it) inherit ~7 s durability.
- The "single source of truth" verified the **model** is byte-identical (0 real-physics field
  diffs) and the drift is **8.5 mm / 10 ticks** — but on an inverted-pendulum biped that drift
  **compounds**: over seconds the trainer and deploy trajectories diverge entirely, which is why
  the *same policy* lives 33 s in one engine and 7 s in the other.
- In the OmniSim deploy, ACH2 falls even faster, and the wide-splay achieved stand (hip_roll
  ±0.41, hip_yaw ±0.38 rad — the champion's balance posture) plus the **intermittent G1 cold-load
  crash** (exit 1, no telemetry) make it worse.

This is the honest answer to *"does the training graph reflect actual robot performance?"*: the
similarity % is faithful **only over the measured window**, and the **trainer does not predict
deploy durability** for this biped.

### 2026-06-24 — H1 corroboration: the gap is the launch IC + obs, not the solver

A sibling H1 walk effort tested the obvious hypothesis raised by this doc: *if the trainer
doesn't predict deploy durability, train IN the deploy solver and close the gap.* **It does
not work.** A Phase-2 trainer (`gpu_newton_h1_walk_trainer.py`, commit `cf200cdc`) fine-tunes
through the **exact** deploy solver (`newton.solvers.SolverMuJoCo`), yet **every** fine-tune
**regressed** the deploy vs the mjwarp-trained champion (run 3): run 3 was **2.03 s / +1.45 m
FORWARD**, Newton-fine-tuned on a fresh-URDF model dropped to **1.58 s / backward**, and
Newton-fine-tuned on a matched dumped-MJCF model dropped further to **0.66 s / backward**.

The matched-MJCF model (commit `da8b171a`) **did** improve fidelity on **survival** (run-3
batched cold-eval **1.26 s → 1.85 s**; at the deploy launch phase the batched survival **2.05 s**
matches the deploy **2.03 s**) — so friction/contacts were a real model gap, exactly as the
byte-match work here closed real model gaps for G1. But the **direction still differs**: run 3 and
its fine-tune are **byte-identical in the batched trainer** (2.05 s / −0.72 m at the launch phase)
yet **diverge in deploy** (2.03 s FWD vs 0.66 s back) — the batched trainer walks **backward**
where the deploy walks **forward**. The cause is **not the solver**: it is the deploy's 0.3 s
settle launch lean + residual velocity (absent in the batched reset) plus the observation pipeline
(world-frame `getVelocity` + finite-diff `qd` vs the trainer's exact MuJoCo-frame `qvel`).

This is the **same phenomenon documented above for G1** — a byte-matched model with divergent
long-horizon/deploy behavior — but the H1 case **localizes a concrete culprit class**: the launch
**initial condition** and the **observation pipeline**, not the physics solver. The path to a
durable (10–20 m) deploy walk is therefore to **align the trainer's launch IC + obs to the
deploy**, not to fine-tune harder on a mismatched IC. Full H1 journal:
h1-walk-rl-journey.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)); canonical status: [rl-current-state.md](rl-current-state.md).

---

## 5. What is and isn't solved

**Solved / real:**
- A numerical, self-tested ghost-similarity metric (no eye-witness needed).
- The diagnosis that the human ghost is a ~67% physical wall and **why**.
- The feasible-ghost method genuinely lifts the **shape** match to **84–88%** and the
  *within-window* stability (128/128 over 3 s) — the residual error collapses as predicted.

**NOT solved:**
- A **durable** (indefinite) ≥80% walk: ACH2 topples ~7 s in the trainer, sooner in deploy.
- The **G1 trainer↔deploy durability gap** (predates this work): G1 walks are flaky everywhere
  (~1–33 s, never indefinite; the byte-matched model still drifts per-tick → compounds).
- The deploy intermittent **cold-load crash**.

---

## 6. Tooling left behind (all committed to main)

| flag / artifact | what it does | commit |
|---|---|---|
| `--eval-ghost-similarity` + fair metric | numerical % similar to the ghost (self-tested) | `8bc5c2e9`,`148e37ae` |
| `--eval-every N` | live in-training ghost-% readout (DR muted, env restored) | `d7ffdd8c` |
| `--mirror-loss W` | L/R mirror-symmetry regularizer (verified; **found to hurt**, default off) | `d693e4c1` |
| `--track-sagittal-w` | per-joint hip_pitch+knee imitation weight (raises sagittal shape) | `72b98f87` |
| `--build-achieved` | extract the feasible "achieved" ghost IN the Newton deploy solver | `cae390b7` |
| `--gait-style achieved` enabled | train/deploy track the feasible ghost (single source of truth) | `00e20c5c`,`8d447e32` |

---

## 7. Honest next-step options (for whoever picks this up)

1. **Durability is the real blocker, not similarity.** To get a durable ≥80% walk, the feasible
   ghost must be extracted from a gait that is durable **in the same engine the policy is
   evaluated/deployed in** — i.e. close the trainer↔deploy gap first, or extract the achieved
   ghost **in the deploy** (`omnisim-bin`) where the champion lives 33 s, not in the trainer where
   it lives 7 s.
2. **Add an explicit durability objective** (alive bonus + push DR) on top of the achieved
   tracker so it doesn't just match a 7 s reference.
3. **Fix the deploy cold-load crash** + the wide-splay-stand spawn height before judging deploy.
4. Accept a **finite-bout** framing (G1 walks ~7–33 s, the known limit) and report ≥80% over that
   bout — but only with the survival number shown next to it.

Recipe details + the deploy launch live in g1-deploy-walk.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) (last section);
the feasible-ghost design in [g1-improved-shadow.md](g1-improved-shadow.md); canonical RL status
in [rl-current-state.md](rl-current-state.md).
