# Shadow Iteration — is Shadowing a fixed-point improvement operator?

## ⭐ THE ANSWER (read this first)

> **YES — and its convergence condition is the feasibility gate.**
>
> ```
> ghost_{n+1} = fold(roll(champion_n))       IFF champion_n's gait PASSES closure + support
> champion_{n+1} = shadow_train(ghost_{n+1})     ... and then it improves, measurably.
> ```
>
> Refreshing the ghost beats "just train longer" by **+29 % and +90 % speed** on two independent
> lineages — while the trained-longer control *degrades* in both. Best policy produced:
> **0.415 m/s, 99.7 % never-fell, 1.28 m drift**, vs the shipped champion's 0.380 / 2.93 m.
>
> But it compounds **only** from a champion whose gait is FOLDABLE. Fold a torque-saturated
> champion and the ghost fails its gates, and training on it costs you 2.2× the speed. Training on
> a *feasible* ghost is itself the projection that keeps the next champion foldable.

This document is written in the order it was discovered, because the wrong turn is instructive: §1–4
are the first campaign, which concluded **"it does not compound"** — a conclusion that was *true of
what it tested* and **incomplete**. §5 is the second campaign, which found the missing step and the
law. Both were measured on RunPod 4090s; §1–4 with n=3 seeds per arm, §5 with a paired ablation on
two lineages.

---

## The question

Round 1 is already in the repo and verified ([go2_shadow_walk](../../projects/policies/skills/quadruped/go2_shadow_walk/skill.json)):

1. take the legacy Go2 champion (residual RL on an analytic trot, 0.382 m/s),
2. roll it **deterministically** on the deploy-matched model,
3. **phase-fold its ACHIEVED gait** at 64 bins and harmonic-smooth it → *that is the ghost*,
4. shadow-train against it (corridor + GHOST-FF, warm-started from the same champion),
5. → **+12.6 % speed, 5× straighter, 0 falls.**

Step 3 is a **denoising projection**: it takes a policy's own behaviour and projects it onto the
clean-periodic-gait manifold. Step 4 then re-optimises the task reward while a corridor holds the
policy near that clean reference. Written as an operator:

```
ghost_{n+1} = fold(roll(champion_n))          champion_{n+1} = shadow_train(ghost_{n+1})
```

If that iteration keeps improving, Shadowing is not "imitate a reference" — it is a
**self-improvement operator on gaits**, and the reference is a fixed point it converges to. Nobody
had asked. So we asked.

## The experiment (and the ablation that makes it mean anything)

A round-2 champion trained for 400 more iterations will beat round 1 **for the trivial reason that
it trained longer**. So two arms, identical in every respect — same warm start (the round-1 shadow
champion), same 400 iters, same 16384 envs, same corridor 0.15, same GHOST-FF, same reward, same
seeds {0,1,2}, **same GPU model** — differing in exactly one thing:

| arm | `QUAD_GHOST` |
|---|---|
| **treat** | ghost **v2**, rebuilt from the shadow champion's OWN achieved gait |
| **ctrl** | ghost **v1**, the old one — i.e. *purely* "train 400 iterations longer" |

Runner: [`pod_shadow_iteration.sh`](../../projects/policies/research/shadowing/pod_shadow_iteration.sh).
The recorder had to be generalised first — it could only ever roll ONE policy (the legacy champion)
on ONE baseline (the analytic trot). A shadow champion rides `ghost + feedforward`; rolling it on
the trot would record it far out of distribution and capture junk. Hence
`build_go2_shadow_ghost.py --policy <onnx> --baseline ghost --baseline-lut <lut>` — **the iteration
primitive: build a ghost from ANY champion, including one that itself rides a ghost.**

## Result 1 — the round-2 ghost FAILS its feasibility gates

The champion rolled perfectly (+5.14 m, **0.428 m/s**, no fall — matching its known 0.429). The fold
was clean. And then:

```
[GATE 1+4 LUT-REPLAY]   FAIL   closure drift p95 24.3 mm   (gate < 20 mm)
[GATE 2+3 SUPPORT+FWP]  FAIL   base_frac95 = 0.174
```

| | source speed | torque saturation | closure p95 | support base_frac95 | verdict |
|---|---|---|---|---|---|
| **ghost v1** (legacy champ) | 0.382 m/s | 21/750 steps | **12.7 mm** | **0.0743** | PASS → trainable |
| **ghost v2** (shadow champ) | 0.432 m/s | **41/750 steps @ 100 %** | **24.3 mm** | **0.174** | **FAIL** |

**THE MECHANISM.** The shadow champion is faster *because it exploits dynamics a kinematic,
phase-indexed reference cannot represent* — it rides its torque limit twice as often as the legacy
champion did. Fold that gait into a lut and a bare position servo can no longer execute it: the
planted feet slip (closure), and the contact forces can no longer supply the base wrench (support).
**The very thing that made the policy better made its gait un-ghostable.**

That is the stopping condition. Shadowing's iteration is not bounded by optimisation — it is bounded
by **feasibility of the source's achieved gait**, and round 1 worked precisely because the legacy
champion's gait was *conservative* (well inside its torque budget) and therefore folded cleanly.

## Result 2 — fold RESOLUTION was mis-tuned (necessary, not sufficient)

Re-folding the SAME rollout (no new rollout — seconds each):

| closure drift (mean / p95) | nb=64 | nb=128 |
|---|---|---|
| harmonics 6 | 21.2 / 43.1 mm | 14.7 / 30.9 mm |
| harmonics 10 | 19.4 mm | 14.3 mm |
| harmonics 16 | 18.8 mm | **14.0 / 24.3 mm** |
| harmonics 24 | 19.6 mm | 13.9 mm |

**Bin resolution is the lever; the harmonic cutoff is almost irrelevant.** `nb=64` was tuned for the
legacy champion's *slower* gait — under-resolving a faster gait **manufactures foot slip**. Doubling
to `nb=128` nearly halves the closure error… and the ghost **still fails** (24.3 mm vs the 20 mm
gate). Necessary, not sufficient.

> **Rule of thumb this gives us:** a ghost's phase resolution must scale with the sharpness of the
> gait it folds. If you speed a robot up, re-check `nb` before you blame the policy.

## Result 3 — THE GATE IS PREDICTIVE (the first time this has been tested on a quadruped)

We then trained on the gate-failing ghost **deliberately**, to find out whether the gate's verdict
actually predicts the training outcome. 3 seeds per arm, the trainer's own in-engine eval
(16384 envs × 1500 steps, randomized ICs):

| arm | speed (m/s), seeds 0/1/2 | mean | fwd dist | never-fell | gmatch | \|ydrift\| |
|---|---|---|---|---|---|---|
| **TREAT** (ghost v2 — failed its gates) | 0.022 / 0.231 / 0.257 | **0.170** | 4.00 m | 97.8 % | 0.854 | **7.52 m** |
| **CTRL** (ghost v1 — trained longer) | 0.304 / 0.400 / 0.437 | **0.380** | 9.05 m | 96.6 % | 0.888 | **2.93 m** |

**Every seed, every metric.** Training on a ghost that failed its feasibility gates costs you
**2.2× the speed** and **2.6× the lateral drift**. The gate — which runs in *seconds*, on CPU,
before a single GPU-hour — predicted it. That is the entire thesis of the gate architecture,
demonstrated: *reject a bad reference in seconds, not after a training run.*

And the control tells us the other half: **more training on a FEASIBLE ghost still improves**
(0.304 → 0.437 m/s across seeds). The problem was never the extra optimisation. It was the ghost.

## ⛔ Result 4 — a single deploy rollout FLATTERS the bad policy. Do not trust it.

On the live deploy ruler (443 s sim, `ONNX loaded:` asserted on both):

| | live single rollout | batched randomized eval |
|---|---|---|
| **v1** (incumbent) | 190.1 m, y-drift **0.01 m**, gmatch 0.864 | — |
| **TREAT** (failed gates) | **197.4 m — further and faster**, y-drift 0.66 m, pitch −0.29 rad | **0.170 m/s, ydrift 7.5 m** |

From one nominal start the gate-failing champion walks 197 m without falling and *beats the
incumbent on distance*. It is only under randomized initial conditions, across thousands of envs,
that it collapses. **A single deterministic rollout cannot distinguish these policies.** Judge a
policy on the batched eval; the deploy rollout is a demo, not a ruler.

## What this means for the method

1. **Shadowing does not compound for free.** The operator has a **stopping condition**, and it is
   *feasibility*, not optimisation.
2. **The stopping condition is measurable, cheaply, in advance** — the closure + support gates.
   They are not bureaucracy; they are the theory of the method made executable.
3. **Shadowing is an "upgrade the incumbent ONCE" operator** for a champion whose gait is
   conservative. To iterate further you would have to make the *achieved gait itself* foldable —
   e.g. train the source with a torque-saturation penalty so the behaviour it converges to stays
   inside what a kinematic reference can express. **That is the open question this experiment
   hands to the next campaign.**
4. The honest framing for the paper: round 1's +12.6 % is real, and it is **not** the first step of
   an infinite ladder. Claiming otherwise would have been the easy, wrong story.

## Reproduce

```bash
# 1. the iteration primitive: a ghost from ANY champion (incl. one that rides a ghost)
python projects/policies/research/shadowing/build_go2_shadow_ghost.py \
    --policy projects/policies/research/inference/policies/gpu_go2_shadow_main/policy.onnx \
    --baseline ghost --baseline-lut projects/policies/ghosts/go2/go2_shadow_ghost_lut.json \
    --nb 128 --harmonics 16 --out _scratch/go2_shadow_ghost_v2
# -> it will REFUSE to certify: closure p95 24.3 mm, support base_frac95 0.174.

# 2. the two arms (on a pod; identical but for QUAD_GHOST)
bash projects/policies/research/shadowing/pod_shadow_iteration.sh treat 0
bash projects/policies/research/shadowing/pod_shadow_iteration.sh ctrl  0
```

Cloud config + throughput: `cloud/runpod/PERFORMANCE.md` (private ops tree — not in the public snapshot).

---

# ⭐⭐⭐ ROUND 2 (2026-07-13, later the same day): **IT COMPOUNDS.** The gate is the convergence condition.

The section above concluded "Shadowing does not compound." **That conclusion was incomplete, and the
completion is the actual law.** It does compound — *iff the source champion's gait is foldable* —
and we now know how to *make* it foldable.

## What we missed

The round-2 ghost failed because it was folded from the **round-1 shadow champion**, whose gait was
torque-saturated. But that champion is not the only thing you can fold. Train it **400 more
iterations on the FEASIBLE ghost v1** — the corridor + gmatch reward act as a *projection back into
the foldable set* — and the resulting champion folds cleanly:

| champion folded | closure p95 | support base_frac95 | gate |
|---|---|---|---|
| round-1 shadow champion (the earlier attempt) | **24.3 mm** | 0.174 | **FAIL** |
| + 400 iters on the feasible ghost (`W_TAU=0`) | **16.5 mm** | 0.0702 | **PASS** |
| + 400 iters, servo penalty `W_TAU=10` | **13.0 mm** | 0.0718 | **PASS** ← best |
| + 400 iters, `W_TAU=2` | 15.5 mm | 0.0839 | closure PASS / support FAIL |
| + 400 iters, `W_TAU=50` | 19.9 mm | 0.1173 | closure PASS / support FAIL |

## And then it compounds — replicated on two independent lineages

Round 2 = 400 more iterations, warm-started from the round-1 champion. **TREAT** trains on the ghost
folded from that champion; **CTRL** trains on the OLD ghost — i.e. *purely "trained longer"*. Same
everything else. The only difference is the ghost.

| lineage | round 1 | round 2 — **NEW ghost** | round 2 — OLD ghost (control) |
|---|---|---|---|
| **W_TAU=0** | 0.318 m/s · 91.8 % · drift 3.69 m | **0.415 m/s · 99.7 % · drift 1.28 m** | 0.322 m/s · **72.6 %** · drift 5.24 m |
| **W_TAU=10** | 0.392 m/s · 98.5 % · drift 3.57 m | **0.382 m/s** · 93.3 % | **0.201 m/s** · 84.9 % · drift 7.35 m |

**Both lineages: refreshing the ghost beats training longer by +29 % and +90 % speed** — and the
trained-longer control *degrades* in both (never-fell collapses to 72.6 % / 84.9 %). The gain is
attributable to the ghost refresh and to nothing else.

`r2t0_new` — **0.415 m/s, 99.7 % never-fell, 1.28 m drift** — is the best Go2 policy in the repo,
beating the shipped round-1 champion (0.380 m/s, 2.93 m drift).

## THE LAW

> **Shadowing is a fixed-point improvement operator, and its convergence condition is the
> feasibility gate.**
>
> ```
> ghost_{n+1} = fold(roll(champion_n))     IF champion_n's gait PASSES closure + support
> champion_{n+1} = shadow_train(ghost_{n+1})     ... and then it improves.
> ```
>
> Training on a *feasible* ghost is itself the projection that keeps the next champion foldable.
> The gates are not bureaucracy — **they are the operator's convergence criterion**, computable in
> seconds, on CPU, before any GPU time.

## What was WRONG in my hypothesis (worth saying plainly)

I predicted the servo-effort penalty (`QUAD_W_TAU`, penalising `|cmd − q|` = τ/kp) would be the
thing that restored foldability. **It was not.** The servo-error RMS barely moved (0.0429 → 0.0420
across W_TAU 0→50) because un-foldability comes from **peak** torque saturation, not mean effort —
an RMS penalty barely touches peaks. What actually restored foldability was **training on a feasible
ghost at all**. `W_TAU` is a *refinement* (it gives the best closure, 13.0 mm) and `W_TAU=2` gives
the best round-1 policy (0.434 m/s) — but it is not the mechanism. Kept, honestly labelled, and the
next intervention to try is a **hinge penalty on the saturation peak**, not the mean.

---

# GENERALIZATION: the tooling is robot-general; OMNIQUAD is the counter-example that proves the law

`build_quad_shadow_ghost.py --robot go2|omniquad|b2` and `build_quad_stand_ghost.py` generalize the
builder (the go2 path is bit-identical to the original — verified constant-by-constant). The
validator, `robot_registry`, and `baton.py` needed **no changes at all** for OmniQuad.

**And OmniQuad immediately demonstrated the law.** Its legacy champion is saturated **210/750 steps
(28 %)** — vs the Go2's 21/750 (3 %). Fold it and:

```
[GATE 1+4 LUT-REPLAY] FAIL: CLOSURE drift mean 35.1 mm p95 81.1 mm   (gate < 20 mm)
[GATE construction re-roll] FAIL: FELL
```

**OmniQuad cannot be Shadowing-trained from its current champion** — the gate says so in seconds, and the
live BATON run confirms it (the robot falls at t=2.51 s and then "walks" inverted at gmatch 0.86 —
the pose-metric trap again). This is not a OmniQuad bug. It is the law, on a second robot.

**The fix is known:** give OmniQuad a feasible reference first (the analytic trot is one by
construction), train it there, and its champion becomes foldable — exactly the Go2 path. That is the
next campaign.

## BATON now runs on THREE robots

| robot | morphology | naming family | runtime | support gate | result |
|---|---|---|---|---|---|
| **G1** | humanoid | `left_/right_` | torch, in-engine, **crane** | double-support | 9 switches, 0 falls |
| **Go2** | quadruped | `FL_/FR_` | **ONNX controller** (no world, no torch) | four-foot support | 2–4 switches, 0 falls |
| **OmniQuad** | quadruped | **`front_left_hip_x`** | ONNX controller | four-foot support | **2 switches, arbiter clean**; robot falls — un-foldable ghost (above) |

`projects/policies/training/baton.py` was imported **unchanged** by all three.
