# Ghost-Anchored Exploration for the Shadowing RL Tracker — a research note

> **Status: research, not shipped.** This documents a literature review + a controlled
> experiment on whether restricting the RL tracker's exploration to states *near the ghost*
> (rather than physically-implausible far states) improves training. Conclusion: it prevents a
> specific reward-hacking collapse at zero cost, but on the *already-solved* quadruped flat-walks
> it does **not** speed up learning — because those tasks are not exploration-limited. The lever's
> predicted payoff is on the **dynamic/contact-rich** open problems, which were not tested here.
> We are leaving it as research. (2026-06-24)

## The question

Can we make the Shadowing tracker more sample-efficient by **not** searching states/configurations
the robot can't reach — concentrating the search **around the ghost states**? Grounded with real
numbers on our four-legged walkers (Spot/Go2/B2), which all walk stably under Shadowing.

## What the pipeline already does, and the one lever it lacks

The quadruped walk trainers (`gpu_mjwarp_{spot,go2,b2}_walk_trainer.py`) already implement **two of
the field's three near-reference levers**:

- **Action-space anchoring** — the policy is a *residual* (±0.15 rad, `RES_SCALE`) on the foot-space
  trot ghost. The search is structurally confined to a tube *in action space* around the ghost.
  (Residual Policy Learning, Silver 2018; OPT-Mimic, ICRA 2023.)
- **Reference-State Initialization (RSI)** — `--seed-gait-pose` resets `q` **and** `qd` onto the
  ghost at a uniformly random phase. This is exactly DeepMimic RSI.

The **missing** lever is **tracking-error / "ghost-tube" Early Termination (ET)**: the only episode
terminator was a *gross fall* (`|roll|,|pitch| > 0.8 rad = 46°`, or base height < 0.30 m). Between
"tracking the ghost" and "fully toppled" lies a wide band of off-manifold states (20–46° tilt
death-spirals; and the **stand-still degenerate** where the policy freezes while the ghost strides
on) that the deploy walk never visits but the trainer still spends samples in. That band is the
"crazy far states."

## Literature grounding (4 parallel deep-research agents, primary sources)

- **DeepMimic (Peng et al., SIGGRAPH 2018, 1804.02717)** — RSI + ET are the two cheap
  near-reference levers. Ablation (normalized return): backflip RSI+ET **0.791**, no-ET **0.379**,
  no-RSI **0.730**. **Crucial caveat for us: on *walk* the ablation barely moves (0.974–0.981 across
  all conditions).** For a periodic gait the payoff of RSI/ET is *sample-efficiency + avoiding the
  stand-still / floor-lying local optima*, **not** peak return.
- **OPT-Mimic (Fuchioka, Xie, van de Panne, ICRA 2023, 2210.01247) — our exact architecture:**
  residual RL on a *trajectory-optimization* ghost, on a quadruped, with RSI + a **tube ET**:
  terminate when `‖x̂−x‖ > 2.5σ` (σ = the reward-term Gaussian width — a unit-free tube).
- **PHC (Luo et al., ICCV 2023)** — "Relaxed ET": terminate at **0.5 m** mean link-distance to the
  reference, but **exclude the dynamics-mismatched feet** → +8.2 pp success. Tube on base/proximal,
  relax the feet.
- **The converged real-robot termination triple** (H2O/OmniH2O/HOVER/UniTracker, 2024–25):
  (mean link dist > **0.5 m**) OR (base height < **0.3 m**) OR (projected-gravity_xy > **0.7–0.8**).
  Our fall gate ≈ the last two; the tube adds the *position/tracking* term.
- **Theory (Kakade–Langford 2002; Agarwal et al., JMLR 2021).** PPO (first-order) iteration
  complexity scales with the **square** of the distribution-mismatch coefficient `‖d^{π*}/μ‖∞`.
  The ghost *is* the restart distribution μ; concentrating exploration near it shrinks that
  coefficient → provably faster convergence — *for the states the ghost covers*.
- **Caveat, stated by 3 of 4 agents:** gate ET on a **generous** "lost-the-ghost / fell-over"
  boundary, **not** a tight tracking distance — too-tight a tube kills the off-reference excursions
  push-recovery needs; **anneal wide→tight** (ASAP 1.5 m→0.3 m, 2502.01143).

## What was built (research prototype; reverted, not shipped)

An opt-in **ghost-tube ET** in `gpu_mjwarp_spot_walk_trainer.py` (default-off ⇒ byte-identical),
DeepMimic-style (pure ET, no extra penalty by default):

- `--tube-rp 0.4` — terminate at 23° tilt (half the 46° fall gate) → prunes the death-spiral band.
- `--tube-prog 0.8` — **forward-progress lag tube**: integrate the velocity deficit `(v* − vx)` after
  a warmup; terminate when the robot has fallen 0.8 m behind the striding ghost. This is the term
  that catches the **stand-still degenerate**: a frozen robot accrues only ~0.19 rad joint error
  (indistinguishable from healthy walk's 0.26), but its *base-position* lag grows without bound —
  the H2O/OPT-Mimic base-position tracking term. The decisive measurement that motivated it:

  | quantity (Spot bare gait) | healthy p99 | healthy max | fall gate |
  |---|---|---|---|
  | joint-RMS err vs ghost | 0.16 | 0.26 | — |
  | \|roll\| | 0.026 | 0.36 | 0.80 |
  | \|pitch\| | 0.10 | 0.80 | 0.80 |
  | frozen-robot joint err vs striding ghost | — | 0.19 | — |

  → a joint tube cannot separate stand-still (0.19) from healthy walk (0.26); a *position-lag* tube
  can. Thresholds sit at ~4× the healthy envelope and ≤ ½ the fall gate — a deliberately **generous**
  tube, per the literature's wide-tube rule.

Plus a **held-out fair eval** (`--eval-curve`): all envs start from the deploy rest-launch,
FALL-ONLY termination (tube off ⇒ fair across arms), deterministic, logging survival/distance/speed
vs env-steps.

## Measured results — Spot walk, 3 seeds, from-scratch, 17.2M env-steps (RTX 5070 Ti)

Held constant: 4096 envs, 350 iters, `--seed-gait-pose --rest-start-frac 0.25 --dr-init-vx-bias 0.3
--rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3`. Arms: **base** (no tube) vs **tube** (`--tube-rp 0.4
--tube-prog 0.8 --tube-warmup 70`). Held-out fair eval, forward speed while alive (m/s, mean±std):

| env-steps | base speed | tube speed | base survFull | tube survFull |
|---|---|---|---|---|
| 2.5M | 0.503±0.044 | 0.498±0.048 | 0% | 0% |
| 5.0M | 0.551±0.037 | 0.525±0.018 | 0% | 0% |
| 7.25M | 0.541±0.015 | 0.496±0.053 | 0% | 0% |
| 9.75M | **0.317±0.014** | **0.467±0.012** | 0% | 0% |
| 12.25M | 0.165±0.126 | 0.461±0.014 | 33% | 0% |
| 17.25M | **0.136±0.077** | **0.436±0.027** | 41% | 0% |

**Durability probe** — warm-start both arms from a common 4.9M walker, +24.6M steps:
- **base**: collapses again — degrades the walk to a **slow survival-shuffle** (0.50 → **0.228 m/s**,
  first-fall 1.3 → 5.3 s) — it trades walking speed to survive longer and farm the alive reward.
- **tube**: **holds a genuine ~0.49 m/s walk the whole way** (no collapse), but stays non-durable
  (first-fall ~1.5 s, survFull 0%).

Throughput identical (~65k env-steps/s both — resets are kinematics-only, **no wall-clock cost**).
~98% of tube-arm episodes were tube-terminated (that fraction of episode-time was off the deploy
manifold). 24 s forward distance was a wash across arms (base stands-slow-survives ≈ tube
walks-fast-falls) — **speed-while-alive is the clean discriminator.**

## Conclusion

1. **The intuition is correct and well-grounded — but the quadruped flat-walks are the wrong place
   to *see* the win.** They already walk; their core learning is not exploration-limited, so the
   tube mostly **prevents reward-hacking** (real, and free) rather than speeding learning. It keeps
   the policy locked onto the target behavior (fast forward walking on the ghost) and rejects both
   degenerate escape hatches — full stand-still *and* the slow-shuffle. It does **not**, on its own,
   produce a durable walk; durability is a separate axis (DR curriculum / trainer↔deploy gap).
   This matches DeepMimic's own flat-walk prediction.
2. **The lever's predicted payoff is where exploration breadth *is* the bottleneck:** the
   dynamic/contact-rich open problems — durable humanoid walk, the G1 sit-stand launch, get-ups,
   hill flat→ramp transitions. DeepMimic's numbers (0.38→0.79 on flips vs ~0 on walk) say the gain
   concentrates exactly there. That experiment was **not** run here and is the natural follow-up.
3. **Cheap RSI upgrade worth noting:** error-weighted (hard-phase) init sampling on top of the
   existing uniform RSI (EGM/GMT report ~19% tracking-error reduction).

## Reproduce

The prototype was opt-in flags on `gpu_mjwarp_spot_walk_trainer.py` (`--tube-rp/--tube-prog/--tube-q/
--tube-h/--tube-warmup/--tube-term` + a held-out `--eval-curve`), since reverted from the working
tree. The mechanism is fully specified above (tube logic in `BatchedSpotWalkEnv.step`; progress-lag
integrates `(v* − vx)·Δt` after warmup; eval env built once with `rest_start_frac=1.0`, tube off,
`max_ep` disabled, deterministic). A near-identical tube (without the progress-lag term) is already
committed on `main` by a parallel work-stream.
