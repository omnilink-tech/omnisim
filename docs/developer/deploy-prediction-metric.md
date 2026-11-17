# The deploy-prediction metric + the control-latency gap

> **TL;DR.** In-engine RL training metrics (episode return, fall count, AMP style
> reward) **do not predict how a policy behaves when actually deployed** — this is
> the source of the recurring "trains great, deploys badly" surprise. The fix is a
> deterministic **deploy-prediction metric** (`deploy_eval` in
> [`g1_amp.py`](../../projects/policies/research/rl_inengine/g1_amp.py)) that
> reproduces the *deployment* condition and reports a hard good/bad number. Building
> it surfaced the root cause behind every failed deploy this program has hit:
> **the live deploy is the batched training rollout PLUS ~4–6 ticks of control
> latency** (a servo-bridge / engine-tick delay) that training never saw. The same
> lever fixes both sides — calibrate the metric *with* latency so it predicts the
> deploy, and *train* with (randomized) latency so the policy survives it.
>
> **Rule for all future in-engine RL here:** gate deploy-readiness on
> `deploy_eval` at the calibrated latency, **never** on a training metric. If you
> change the deploy control path, re-calibrate the latency against a live-deploy sweep.

## Why training metrics lie about deployment

The batched training rollout and the live deploy differ in three ways that make a
policy look durable in training yet fall in seconds when deployed:

| | Training rollout | Live deploy |
|---|---|---|
| Action | **stochastic** (sampled) | **deterministic mean** |
| Horizon | short (`T=32`), reset-heavy | long, continuous (a whole demo) |
| Control path | direct `ctrl` write + my `mjw.step` loop | `joint_target_pos` → Newton servo bridge → engine tick |

A high episode return can come from rich per-step reward over *short* survival; a
stochastic policy can lean on its exploration noise; and — the big one — the live
control path is **delayed** relative to the batched one.

## The metric: `deploy_eval`

`deploy_eval` (nested in `_amp_train_gpu`) reproduces the *deploy* condition in the
fast batched engine:

- **deterministic** — runs the policy **mean** action (no sampling), like deploy;
- **long horizon** — `EVAL_H` steps (~700–1500 = a real demo length);
- **fixed seed + deploy-like IC** — reproducible across checkpoints;
- runs on the K=2048 batched rollout, so it's cheap (a few seconds).

It reports, per checkpoint:

| field | meaning |
|---|---|
| **`surv`** | fraction of the horizon survived before falling — the #1 number |
| **`fall`** | fraction of worlds that fell within the horizon |
| **`sat`**  | action-saturation fraction — predicts over-drive / collapse |
| **`fidel`**| motion fidelity under *deterministic* execution (AMP style reward) |
| **`drift`**| lateral RMS displacement (for "stay-in-place" motions) |

Two entry points:
- **periodic, during training** — set `EVAL_EVERY=N`; the trainer logs
  `DEPLOY-EVAL it=… surv=… fall=… sat=… fidel=… drift=…` every `N` iters (it
  clobbers the rollout buffers, so the trainer re-seeds afterward).
- **standalone, on a saved checkpoint** — set `EVAL_ONLY=1` with `RES_POLICY=<ckpt>`;
  it loads, evals once, logs `DEPLOY-EVAL-ONLY …`, and returns.

## The discovery: the live deploy = batched rollout + ~4–6 ticks of latency

When first built, the metric **disagreed with the live deploy**: it reported
`surv≈0.90` for a checkpoint whose live deploy fell almost immediately
(`surv≈0.05`). That proved the **batched rollout — which training *and* the naive
eval use — is systematically *easier* than the live deploy**, which is the root of
the whole "trains great, deploys badly" pattern.

Adding a synthetic control latency to the eval (`AMP_CTRL_LAT=N`: apply the
commanded target delayed by `N` ticks) calibrates it to the live deploy:

| latency | eval `surv` | eval `fall` |
|---|---|---|
| 0 | 0.96 | 0.12 |
| 1 | 0.88 | 0.26 |
| 2 | 0.63 | 0.58 |
| **4** | **0.22** | **0.96** |
| 6 | 0.19 | 0.99 |
| **live deploy** | **~0.05–0.12** | **~1.0** |

So the live deploy behaves like the batched rollout **with ~4–6 ticks of control
latency** — the Newton servo-bridge / engine-tick delay. (The earlier parity probe's
`live_ctrl[t] == target[t-1]` was a real hint, not a read-timing artifact.) A policy
trained latency-free is not robust to that delay and collapses in deployment.

## The unified fix (metric work == durability work)

Make the batched setup as hard as the live deploy:

1. **Trustworthy metric** — run `deploy_eval` at the calibrated latency
   (`AMP_CTRL_LAT≈4–6`) so it predicts the live deploy.
2. **Durability** — **train** with control latency so the policy learns to handle the
   delay. `AMP_CTRL_LAT=N` applies the same delay in the training rollout (the obs and
   action-rate still use the *commanded* action; only the *applied* `ctrl` is
   delayed). ⚠️ A **fixed** latency lets the policy overfit that exact value (the
   metric then over-predicts), so **randomize** it: `AMP_LAT_MAX=M` draws the latency
   uniformly in `[0,M]` per iteration. Evaluate conservatively at the high end.

### Knobs

`EVAL_EVERY` `EVAL_H` `EVAL_IC` `EVAL_ONLY` · `AMP_CTRL_LAT` (fixed latency, train+eval)
· `AMP_LAT_MAX` (randomized training latency) · plus the durability levers the metric
guides: `RES_ACT_PEN` / `W_ARATE` (cut the `sat` it flags), `DISC_LR` (calm GAN
oscillation visible in `surv`), `OBS_NOISE` / `IC_RAND_*` (robustness), `CKPT_EVERY`
(keep the highest-`surv` checkpoint).

## Honest status (2026-06-30)

- **The metric is built, calibrated, and working** — it correctly flags
  non-durable policies that training metrics call "great," and it gives actionable
  diagnostics (e.g. high `sat` → the policy over-drives). This is the deliverable.
- **Durability is not yet achieved.** Training with latency gave a real but small
  deploy gain (survival t81→t161); the randomized-DR version oscillates/plateaus at
  `surv@lat6 ≈ 0.4–0.5` (not durable). A truly durable from-scratch policy is a
  multi-day investment (more stable GAN, likely recurrence/LSTM, physics DR, far more
  compute) — but it is now **measured**, not guessed.

See also: [train-deploy-gap.md](train-deploy-gap.md) (the maintainer reframe: the
train↔deploy micro-differences are a robustness *feature*),
[closed-loop-chaos-diagnostic.md](closed-loop-chaos-diagnostic.md).
