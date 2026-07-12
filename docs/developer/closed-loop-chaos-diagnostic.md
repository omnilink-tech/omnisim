# The closed-loop BUG-vs-CHAOS diagnostic (run this when train ≠ deploy)

> **When to reach for this.** A policy behaves differently in the trainer than in
> deploy and you don't know why. Before you "fix the train→deploy gap," run this
> measurement. It tells you which of two fundamentally different things you are
> looking at:
>
> - a **real bug** in the train/deploy pipeline (wrong frame, wrong gain, wrong
>   constant, obs computed differently) — **fixable, go fix it**; or
> - **physical chaos** — the irreducible exponential divergence of an unstable
>   system (a balancing biped), which is **not a bug** and cannot be removed by
>   making the pipelines "more identical."
>
> Historically this distinction is where weeks were lost: a deploy fall was blamed
> on "the physics is wrong" when the physics was identical and the real story was
> either a one-line obs bug or plain chaos. **This tool ends that guessing.**

Tool: [`projects/policies/research/training/closed_loop_parity_compare.py`](../../projects/policies/research/training/closed_loop_parity_compare.py)

---

## The idea in one paragraph

Run the **same policy** with the **same control law** on **two sides** — the
trainer's physics path and the real deploy binary — logging, per control tick, both
the **observation vector** the policy received and the resulting **state** (joint
angles + base pose). Then diff them tick by tick and read the *shape* of how they
diverge. The shape is the diagnosis.

## The three signatures

| Verdict | Signature | Meaning | What to do |
|--|--|--|--|
| **`[BUG]`** | State diverges **from tick ~1**, before chaos can grow | A gross physics / initial-condition / model difference | Find & fix it. The per-component obs diff names the channel. |
| **`[BUG?]`** | State looks chaos-like, **but an obs channel is anomalous in the early window** (e.g. a velocity/rate term off by ~2 while the pose matches) | An **obs-pipeline bug hiding under the chaos** — it corrupts the obs at tick 1 but takes a few ticks to bend the trajectory | Make the obs construction identical on both sides, re-run; only then trust a CHAOS verdict. |
| **`[CHAOS]`** | Two match to the float floor for the first ~0.3 s, then diverge **exponentially** | **Not a bug.** Intrinsic instability (positive Lyapunov exponent): float32 rounding + tiny obs noise amplified | Not closable. Reduce with policy **robustness** (domain randomization / margin), or accept it. |
| **`[MATCH]`** | Stays matched (≈1e-5) the whole run | Gold-standard parity — trainer and deploy are the same system | Nothing; this is the welded-base control result. |

The **welded-base control** is how you make the verdict airtight: pin the base and
the inverted-pendulum instability is gone, so a clean pipeline returns `[MATCH]` to
~1e-5. If the welded lane matches but the free lane diverges late, the free-lane
divergence is *proven* to be chaos, not a model gap. (Caveat: an obs term that is
zero when the base is still — like base angular velocity — won't show on the welded
lane; that is why the free-lane per-component obs diff and the `[BUG?]` caution
exist.)

## Worked example — the G1 stand (2026-06-27)

The from-scratch G1 stand stood 32 s in deploy but "slid 4 m." Was the slide a
deploy bug? The diagnostic, same policy, certified trainer vs real binary:

```
  tick   t(s)   |d_base_xy|   ...
     0   0.00   5.77e-03            matched
    10   0.16   2.72e-03            matched  (float floor)
    24   0.38   9.26e-02            parting
    48   0.77   3.48e-01            diverging
    96   1.54   9.26e-01            far apart
   199   3.18   2.71e+00            unrelated paths

VERDICT:  [CHAOS] PIPELINE clean -> the divergence is PHYSICAL CHAOS, not a bug.
          Matched to 0.0056 m for the first 10 ticks (0.16 s), then diverged
          EXPONENTIALLY -- e-folding every 17 ticks (0.27 s), reaching 2.71 m.
```

Matched to **5 mm for 0.3 s**, then exponential (e-folding every ~0.27 s). That is
the chaos fingerprint. The slide is not a pipeline bug — it is that a free biped is
chaotic, and the policy's *walk* (its balancing strategy) lands in a different place
each run. The same tool, run on the **earlier** traces that still had the
angular-velocity obs bug, instead returns `[BUG?]` and points straight at the
`rate` channel (off by 2.1 rad/s at tick 1) — exactly the bug that was then fixed
([binary-parity-probe.md](binary-parity-probe.md) finding 4).

## Why chaos is not a gap you can close

A free-standing / balancing robot is an **unstable** dynamical system — a positive
Lyapunov exponent. Any difference between two runs, no matter how tiny (the last bit
of a float32, a 3 mm finite-difference in an obs term), is **amplified
exponentially**. Two runs that start 1e-6 apart are 1 m apart a couple of seconds
later. This is a property of the *physics*, not of our code. You cannot make two
free-base biped rollouts track forever, in any simulator, ever.

What you *can* do:
- **Verify there is no real bug** underneath (this tool: get `[CHAOS]`/`[MATCH]`, not `[BUG]`).
- **Give the policy more stability margin** — a stand with a wider basin, or a
  self-correcting gait (a walk re-stabilises every footstep), tolerates the divergence;
  a razor-margin policy tips.
- **Confirm with the welded lane** — `[MATCH]` there proves the engine + model + obs
  are the same system and the free-lane divergence is purely the instability.

> ⚠️ **Domain randomization does NOT help a same-engine sim→sim chaos gap — it
> measurably HURTS** (measured 2026-06-27 on the G1 stand). Sweeping DR strength: heavy
> DR fell at **2.6 s** in deploy, light DR fell at **19.6 s**, the *lightest* DR (dr2)
> stood **32 s** — monotonic, more DR = worse. The reason: DR robustifies against a
> *domain shift* (different mass/friction/latency — the sim→**hardware** gap). Between
> the trainer and the deploy *binary* there is **no domain shift** (same engine, proven
> `[MATCH]` welded); the only difference is chaos. So DR just teaches the policy to brace
> against perturbations that aren't there at deploy, making it over-reactive and tipping
> it *sooner*. Reserve DR for real sim→hardware transfer; for the chaos gap the lever is
> controller margin, not DR.

## How to run it

1. Run the **same** ONNX policy with the **same** control law + **same** obs
   construction on both sides, each dumping a per-tick trace
   (`{"ticks":[{"k","phase","q","base_pos","base_rot","obs"}, ...]}`). For the G1:
   - trainer : [`g1_policy_eval_addurdf.py`](../../projects/policies/research/training/g1_policy_eval_addurdf.py) (certified `add_urdf`+SolverMuJoCo)
   - deploy  : [`g1_policy_probe`](../../projects/policies/research/controllers/g1_policy_probe/g1_policy_probe.py) controller in the real `omnisim-bin`
   (launcher: [`scripts/dev/run_g1_policy_probe.ps1`](../../scripts/dev/run_g1_policy_probe.ps1))
2. Diff them:
   ```bash
   python projects/policies/research/training/closed_loop_parity_compare.py \
       --trainer _scratch/parity/cl_trainer_free.json \
       --deploy  _scratch/parity/cl_deploy_free.json
   ```
3. Read the verdict. If `[BUG]`/`[BUG?]`, fix the named channel and repeat. If
   `[CHAOS]`/`[MATCH]`, the pipeline is sound — the problem (if any) is policy
   robustness, not parity.

**The control law and obs construction MUST be byte-identical on both sides**, or
you are measuring *that* difference instead of chaos. (`obs` logging is optional —
without it you still get the state-divergence `[BUG]`/`[CHAOS]`/`[MATCH]` verdict,
just not the per-component obs breakdown or the `[BUG?]` caution.) The tool is
robot-agnostic: any two traces in the schema work.

## Standard practice

- Treat this as the **first** step whenever a policy transfers worse than expected.
  Don't theorise about the physics until you've classified the divergence.
- A new RL deploy target (new robot, new controller) should ship a free-lane and a
  welded-lane trace pair, so the `[MATCH]`/`[CHAOS]` baseline is on record.
- See also [binary-parity-probe.md](binary-parity-probe.md) (the open-loop physics
  probe — certifies the engine; this closed-loop tool certifies the obs pipeline and
  separates chaos) and [rl-current-state.md](rl-current-state.md) (canonical status).
