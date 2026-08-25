# G1 walk endurance — measured, 2026-08-01

**Why this exists.** [`capability-ladder-plan.md`](capability-ladder-plan.md) §9 Q1 requires that
T4's ten-metre bar be *measured* against what this tree ships **before** the pre-registration
freeze, and forbids lowering it afterwards. §2 T4 recorded the prior expectation plainly:

> *"The flagship decent walker measures **0.120 m/s** … Ten metres at that speed is **≈ 83 s of
> continuous walking**, and **no continuous G1 bout of that length is recorded anywhere in this
> tree that I found**. So even the supported cell may come back `not_achieved` on distance."*

**That expectation is wrong, and this document is the measurement that refutes it.** The shipped
flagship crosses 10.0 m at **t = 82.9 s** — within 0.5 % of the arithmetic prediction — does not
fall, and keeps walking until it hits a wall. ⚠️ **Every number below is on the λ = 0.9
weight-bearing balance harness. This is not a free-standing walk, and no sentence in this file
may be quoted without that clause** ([`AGENTS.md`](../../AGENTS.md) humanoid disclosure rule).

---

## 1. Environment (recorded, not assumed)

| | |
|---|---|
| **Machine** | `9722d23d12a3` — host `hc385771a14`, AMD64 Family 25 (16 cores), **NVIDIA RTX 3060 Laptop GPU** (driver 596.36) |
| **Repo** | `O:\omnisim`, `main` @ **`0f9f07f1`**, OmniSim 5.5.1, Windows 11, Python 3.12.9 |
| **`omnisim doctor`** | `build  engine + libController compatible` — the IPC-nonce ABI gate **passed** before any run (a stale libController silently hangs every controller at zero ticks) |
| **Engine binary** | `msys64/mingw64/bin/omnisim-bin.exe` sha256 `1d7186d8f24c3d34087b182efd3c83f7e982deef4b304b0ff58eb60dcec8d052` |
| **libController** | `lib/controller/Controller.dll` sha256 `417dd4d28bfd6a91183c6eab7c7de31627a963b30c33ba9c889a4afba864da83` |
| **Policy under test** | `projects/policies/training/runs/wr_decent_walker.pt` sha256 `4d37899884d2944f48d7388a5fbd5c728e125aa7eeb9f95f84e06f820357e2f1` (asserted in-run: `POLICY LOADED: …` line, every run) |
| **Ghost** | `projects/policies/ghosts/g1/ghost_official_full_v3_lut.json` sha256 `a6ed6151467621a2c6324a4454b904be9d0a95ab3a10bf69433f9143eb2d1203` |
| **Stack** | newton 1.2.0, warp-lang 1.13.0, mujoco 3.8.1, mujoco_warp 3.8.0.3, torch 2.5.1+cu121, onnxruntime 1.26.0 |

**Backend provenance — from the sidecar, never scraped from the log.** Every run wrote
`<OMNISIM_LOG_PATH>.newton.json` containing, byte-identically in all six runs:

```json
{"backend":"newton","degraded":false,"finalised":true,"solver":"MuJoCo (mujoco_warp, WorldInfo.newtonSolver)"}
```

The deploy lane also passes `--require-newton`, so at the time of the run a silent ODE
fallback would have exited non-zero rather than degraded. `degraded:false` rules out an XPBD
fallback. (⚠ 2026-08-08: that guarantee is now **structural**, not a property of the flag —
`bdc02139` deleted the ODE backend and `94f04222` removed XPBD, so neither fallback exists to
be caught. The sidecar evidence above is unaffected.)

**Environment caveat.** `env_fingerprint.py` warns that this machine carries a machine-scope
`WEBOTS_HOME='C:\Program Files\Webots'` pointing at a directory that does not exist. Runs resolve
`OMNISIM_HOME` module-relative to this checkout and are unaffected; recorded here so the
fingerprint is not read as clean when it is not.

---

## 2. What was run, and what was *not* touched

The measured artifact is the shipped flagship launcher, invoked with no edits:

```powershell
powershell -ExecutionPolicy Bypass -File projects\policies\worlds\run_g1_decent_walker.ps1 -Headless -Duration 150
```

No policy, checkpoint, ghost, launcher, recipe or world was modified for runs 1–4. Run 4 adds one
**logging-only** environment variable, `CRANE_LOG=1`, which makes the deploy hook print the
harness wrench it was already applying; it changes no physics term. Runs 5–6 are a clearly-labelled
**supplementary probe** on a modified *world* (§5) — same policy, same harness, same corridors.

### Harness settings actually in force (read from the launcher, lines 69–71, not assumed)

```
HARNESS_LAM0=0.9   HARNESS_KP=600  HARNESS_KD=60   HARNESS_FY=400
HARNESS_KZ=2000    HARNESS_DZ=150  HARNESS_Z0=0.70 HARNESS_ATT_GHOST=1
VX_MAX=0.45
```

Confirmed live by the deploy banner: `DEPLOY HARNESS (add_body_force): lam=0.90 newton_body=5`.
The engine-side law ([`g1_walk_recipe.py`](../../projects/policies/training/g1_walk_recipe.py)
L5965 ff.): vertical `fz = clamp(0, 700, λ·(KZ·(Z0 − z) − DZ·ż))` — **upward-only**, capped at
**700 N ≈ 2.09 × the G1's 34.1 kg (334.5 N)**; attitude `tx/ty` a body-frame PD rotated to world,
clamped **±350 N·m**; lateral catch `fy`; `fx ≡ 0` by construction — *the harness applies no
forward force, so forward travel is footwork*; `tz = 0` because `HARNESS_KYAW` defaults to 0 and
no heading target is set on a straight walk.

---

## 3. Per-run results

Telemetry: the deploy hook's own `walk-recipe deploy t=… x=… y=… z=… roll=… pit=…` line
(every 20 control ticks; `engine_control_dt = 0.0160 s`, echoed by the run banner). Analyzer:
`_scratch/g1_endurance/analyze.py` (read-only). Fall criteria = the harness cut-out thresholds
(|roll| > 0.6, |pitch| > 0.6, base z < 0.35) **plus** the recipe's own `BATON_FALL_Z = 0.45`
pelvis gate. All distances are base-frame world displacement from the run's own first sample.

| run | world | wall s | sim s | **fell?** | net Δx (m) | net disp (m) | path len (m) | **t to 10.0 m** | mean speed over run | max abs roll | min base z |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | shipped 26×14 | 202 | 312.6 | **no** | 12.87 | 14.59 | 22.44 | **82.88 s** | 0.047 m/s | 0.085 rad | 0.735 |
| 2 | shipped 26×14 | 202 | 311.7 | **no** | 12.87 | 14.58 | 22.67 | **82.88 s** | 0.047 m/s | 0.081 rad | 0.736 |
| 3 | shipped 26×14 | 182 | 314.2 | **no** | 12.89 | 14.62 | 22.78 | **82.88 s** | 0.047 m/s | 0.082 rad | 0.735 |
| 4 | shipped 26×14, `CRANE_LOG=1` | 182 | 282.6 | **no** | 12.90 | 14.62 | 21.34 | **82.88 s** | 0.052 m/s | 0.081 rad | 0.736 |
| 5 | probe 60×14 (§5) | 384 | 445.4 | **no** | **29.91** | 30.68 | 39.20 | **82.88 s** | 0.069 m/s | 0.079 rad | 0.736 |
| 6 | probe 60×14 (§5) | 364 | 434.6 | **no** | **29.90** | 30.68 | 38.56 | **82.88 s** | 0.071 m/s | 0.080 rad | 0.734 |

**6 / 6 runs: no fall. 2101 s (35.0 min) of cumulative sim time on the harness, zero falls.**
Every run: `[headless] Results: 0 errors, 5 warnings … PASS`, on **launch attempt 1 of 5** — the
documented ~1-in-3 launch flake did not appear once, and true-headless G1 runs did **not** crash
(contrary to the older operational note; the GUI fallback was never needed).

**The "mean speed over run" column is not the walking speed** and must not be quoted as one. It
averages the whole recording *including* the post-wall period. The walking speed is §4.

---

## 4. The shape of the run: it walks at 0.12 m/s until a wall stops it

Run 1, sampled every ~20 s (identical in runs 2–4):

```
 t_s     x      y     net    z     yaw   segment speed
   0.0   0.02   0.00   0.00 0.778   0.03   —
  20.2   2.44  -0.04   2.42 0.740   0.14   0.1201
  40.3   4.87  -0.09   4.85 0.748   0.07   0.1206
  60.5   7.33  -0.13   7.31 0.747   0.03   0.1220
  80.7   9.77  -0.17   9.75 0.747   0.07   0.1210
 100.8  12.10  -0.31  12.08 0.753   0.05   0.1158
 121.0  12.90  -0.81  12.91 0.740  -0.19   0.0468   <-- x pinned at 12.90
 …
 302.4  12.91  -6.88  14.61 0.760   0.00   0.0018   <-- pinned in the corner
```

`x` freezes at **12.90 m** and stays there for 200 s while `y` runs out to **−6.88 m**. The
shipped world is `RectangleArena { floorSize 26 14 }` — centred at the origin, so its walls are at
**x = ±13.0** and **y = ±7.0**, with `wallHeight 0.05` (the G1's foot bodies ride at z ≈ 0.04–0.07,
so a 5 cm lip blocks them). **The run terminated on world geometry, not on a fall.** The robot then
kept locomoting sideways along the wall for another 200 s and never went down.

Gait quality over the clean forward-walking window (before wall contact), from the recipe's own
per-cycle `CYCLE-DIAG`:

| | run 4 (t ≤ 107 s) | runs 5 & 6 (t ≤ 254 s) |
|---|---|---|
| gait cycles | 134 | 318 |
| per-cycle speed, p50 (min–max) | 0.115 (0.066–0.141) m/s | 0.114 (0.066–0.141) m/s |
| forward travel per 0.8 s cycle, p50 | 0.0918 m | 0.0910 m |
| **backward samples** | **0.0 %** | **0.0 %** |
| stance-foot slip, loaded foot, p50 per cycle | 0.110 m | 0.103 m |

The 14–20 % "backward sample" figure visible in a whole-run summary comes entirely from the
post-wall sliding phase; **during the walk itself there are no backward samples**, matching the
2026-07-17 audit result in [`rl-current-state.md`](rl-current-state.md). ⚠️ Stance-foot slip is
**≈ the same magnitude as the per-cycle advance** — worth stating in any gait-quality claim, and a
known open front, but it is not the subject of this measurement.

### Harness support actually applied (the T4 `external_support` channel)

`CRANE_LOG=1`, over the clean walking window, body weight m·g = 334.5 N:

| channel | peak | mean abs | fraction of window non-zero |
|---|---|---|---|
| **fx** (forward) | **0.00 N** | 0.00 | **0 %** — zero by construction; all forward travel is footwork |
| **fz** (vertical, "carry") | **0.00 N** = **0.000 × body weight** | 0.00 | **0 %** |
| fy (lateral catch) | 36.8 N = **0.110 × body weight** | 10.4–10.8 N | 100 % |
| tx (roll attitude) | **69.2 N·m** | 22.4–23.1 N·m | 100 % |
| ty (pitch attitude) | 21.4 N·m | 7.2–7.7 N·m | 100 % |
| tz (yaw) | **0.0 N·m** | 0.00 | 0 % |

**The weight-bearing channel was idle for the entire walk.** `HARNESS_Z0 = 0.70` and the base z
never dropped below 0.734, so `KZ·(Z0 − z)` stayed negative and the upward-only clamp held `fz` at
zero. ⚠️ **This does not make the walk free-standing and must not be reported that way.** The
attitude spring — 600 N·m/rad, capacity ±350 N·m — was non-zero **100 %** of the window at a
**69.2 N·m peak** on a 34 kg robot, and the fact that the pelvis never sank below `Z0` is plausibly
*a consequence of* that spring holding it upright. What the measurement narrows is the honest
claim: the crane's **capacity** is 2.09 × body weight, its **use on this run** was 0 N vertical and
up to 69.2 N·m of attitude authority plus a 0.11 × body-weight lateral catch.

Against the ladder's own `T4-unsupported` test (peak non-gravitational non-contact force
≤ 0.02 × m·g = 6.69 N **and** ≤ 2 N·m): this run is **5.5× over the force threshold and 34.6× over
the torque threshold**. It is decisively a `T4-supported` cell, and the numbers above are exactly
what §2 T4 requires be printed inside it.

---

## 5. Supplementary probe: was 12.9 m the policy or the arena?

**The arena.** One field changed — the shipped world copied with `floorSize 26 14` → `60 14`
(walls at x = ±30 instead of ±13), same policy, same checkpoint, same harness, same corridors, same
launcher env, run through a scratch mirror of the flagship launcher. Reproduce with:

```bash
sed 's/floorSize 26 14/floorSize 60 14/' projects/policies/worlds/g1_walk_puppet.omniworld \
  > projects/policies/worlds/_tmp_g1_endurance_long.wbt
```

*(the temporary world was deleted after the runs; nothing was committed)*

| milestone | run 5 | run 6 | mean forward speed to that point |
|---|---|---|---|
| x ≥ 10 m | t = 82.90 s | t = 82.90 s | 0.1208 m/s |
| x ≥ 20 m | t = 170.58 s | t = 170.58 s | 0.1172 m/s |
| x ≥ 29 m | t = 249.94 s | t = 249.94 s | 0.1161 m/s |
| **x ≥ 29.5 m (new wall)** | **t = 254.10 s** | **t = 254.10 s** | **0.1161 m/s** |
| lateral drift at 29.5 m | y = −1.88 m | y = −1.88 m | 6.4 % of forward distance |
| fell? | **no** (445 s sim) | **no** (435 s sim) | — |

**The honest maximum observed: 29.5 m of continuous forward walking in 254 s, no fall, and the run
still ended on a wall rather than on the policy** — on the λ = 0.9 weight-bearing balance harness,
with the vertical channel measuring 0 N throughout. The policy's true distance ceiling was **not
reached by this measurement**; 29.5 m is a lower bound, not a limit.

---

## 6. Distribution, and why it is tighter than three samples should be

`t` to 10.0 m was **82.88 s in all six runs** — the sampling resolution of the telemetry (0.32 s).
Mid-run states are identical to two decimals between runs 2, 3, 5 and 6; run 1 differs by 0.02 m at
t = 95.7 s. So the spread of this measurement is essentially zero, and **three runs here are
reproducibility checks, not three independent samples** — the same caveat the tree already applies
to the Go2 head-to-head. Two structural reasons: the flagship sets `WALK_MJW_RESET=0`, so there is
no handoff reset and no IC jitter at deploy (every run starts from the identical settled pose), and
the walk is a stable limit cycle that re-converges. The small run-1 divergence is consistent with
the documented non-bitwise behaviour of the GPU `mujoco_warp` path
([`determinism-scope.md`](../benchmarks/determinism-scope.md)) — the runs are *not* bitwise
identical, they are dynamically attracted to the same trajectory.

**Consequence for the ladder:** a T4 cell run on this rig will reproduce, but "3/3" here is weaker
evidence than "3/3" on a stochastic policy. It says the artifact is reliable, not that the
distribution is wide and we sampled its good tail.

---

## 7. Verdict

**(a) — comfortably achievable.**

- 10.0 m net base displacement is reached at **t = 82.9 s** on the shipped flagship, in the shipped
  world, with the shipped checkpoint, **6 / 6 runs, zero falls**.
- 0.1208 m/s to the 10 m mark — the 0.120 m/s the tree already publishes, sustained rather than
  measured over a 15 s window.
- Margin in the shipped world: it reaches **12.9 m** before the arena wall — a **29 % headroom**
  over the bar.
- Margin of the artifact itself: **29.5 m in 254 s** in a longer arena, still no fall, still ended
  on geometry.
- **Honest maximum observed: 29.5 m / 254 s of continuous walking; longest continuous upright bout
  445 s; zero falls in 35.0 minutes of cumulative sim.** All on the λ = 0.9 weight-bearing balance
  harness (0 N vertical applied, 69.2 N·m peak attitude, 100 % of window).

**Reconciliation with the tree's prior record.** The "finite ~34 s bout / 5.9 m, and the 297 m
zero-falls claim does not reproduce" note predates the **2026-07-17 deploy audit**, which
[`rl-current-state.md`](rl-current-state.md) records as fixing the slow half-cadence launch and
adding the corridor-bounded, foot-generated hip-yaw heading trim (`FOOT_HEADING_LOCK=1`, set by the
flagship launcher). That trim is what keeps forward travel from curling into a circle — measured
here as **1.88 m of lateral drift over 29.5 m forward (6.4 %)**. The old finite-bout record is
superseded for this configuration; it should not be cited against T4.

**What this does *not* establish:** nothing about a free-standing (λ = 0) walk, which remains an
open problem; nothing about rough ground, turning under load, or a different robot; and nothing
about a distance beyond 29.5 m, which was never probed to failure.

---

## 8. What this means for the ladder's T4 bar

§9 Q1 asked for a number before the freeze. The number says the bar is not the binding constraint
we feared. Three options, **not** chosen here — the owner and the plan's author decide:

### Option 1 — keep T4 at 10.0 m

**Consequence:** our own `T4-supported` cell is now expected to read `achieved`, not
`not_achieved`, with a support number of roughly
`achieved 3/3 (supported: peak vertical 0.00 × body weight for 0 % of window, peak lateral 0.11 ×
body weight, peak attitude torque 69.2 N·m, non-zero support 100 % of window; reuse_class: assembled)`.

*For:* it is the number that was pre-registered before it was measured, which is the whole point of
pre-registration; it is a round, defensible, roboticist-legible distance; and it keeps the §3.4
pessimism discipline intact — the plan expected to lose this cell and did not, which is the *good*
direction for a pre-registered bar to move. `T4-unsupported` still fails, so the tier still
publishes a real ✗ in our own column.
*Against:* 10 m is now known to sit at 34 % of the artifact's demonstrated reach, so the tier is
less discriminating than it looks — a column that can walk at all will likely clear it.

### Option 2 — state T4 at a distance the tree can actually reach

E.g. 25 m or 30 m, from the 29.5 m measurement.

⚠️ **This is a self-serving choice and must be labelled as one, in the published tier text, in the
same sentence as the number.** Setting a bar from our own capability *after measuring it* is
exactly what §9 Q1 forbids in the downward direction, and the upward direction is not automatically
innocent: a bar at 25–30 m is one this tree clears and most columns' out-of-the-box locomotion
samples may not, which is rigging by selection of the axis rather than of the threshold.
*For:* it discriminates; the measurement exists and is reproducible.
*Against:* it inverts the pre-registration guarantee, and the honest disclosure required
("we set this bar after measuring our own robot at 29.5 m") materially weakens the tier's
credibility — likely more than the extra discrimination is worth.

### Option 3 — split T4 into distance tiers

`T4.a = 10 m`, `T4.b = 25 m`, each with its own support cell, graded from the *same* run.

*For:* costs one extra threshold check on a recording that is already being made, so it is nearly
free; it preserves the pre-registered 10 m exactly as written *and* captures the headroom; and it
degrades gracefully — a column that manages 11 m gets a real, non-binary result instead of a ✓ that
hides how close it came. It also front-loads the arena problem (§below) into the task definition,
where it belongs.
*Against:* more cells to review, and `T4.b` inherits Option 2's labelling problem in weaker form —
25 m was still chosen after seeing 29.5 m, and that should be disclosed even in this framing.

### A build note the tier needs regardless of which option wins

**The world, not the policy, ended every single run.** The shipped puppet arena
(`floorSize 26 14`) caps forward travel at **~12.9 m**, i.e. only 29 % above a 10 m bar and below
any bar of 15 m or more. Whatever T4 finally says:

1. The T4 recorder must **fail loudly on arena contact** rather than record a plateau as a stall or
   a fall — all six runs here would have been misread as "the walk degrades to 0.037 m/s after
   ~110 s" by a grader that did not know where the walls were.
2. Every column's T4 task needs a **stated minimum free run-up ≥ 1.5 × the bar**, and OmniSim's own
   column needs a world authored to it (the shipped one does not satisfy that at 10 m by much, and
   fails it outright at 15 m).
3. This also affects §9 Q2: our `external_support` attestation is real and per-channel
   (`fx/fy/fz/tx/ty/tz`, per tick, from the applied wrench itself), which is a strong position for
   the assertion T3.4 and T4 both rest on — but it is only strong if the run it attests is not
   silently geometry-bound.

---

## 9. Reproducing this

```powershell
# environment gate first -- a stale libController hangs every controller at zero ticks
python -m omnisim doctor
python projects/policies/common/env_fingerprint.py

# the measured artifact, unmodified (repeat; ~200 s wall each, ~312 s sim each)
powershell -ExecutionPolicy Bypass -File projects\policies\worlds\run_g1_decent_walker.ps1 -Headless -Duration 150

# add the support attestation (logging only; changes no physics term)
$env:CRANE_LOG="1"   # then re-run the line above
```

Telemetry lands in `_scratch/foot_redesign/flagshipdemo_rl.txt`; the backend verdict sidecar in
`_scratch/foot_redesign/flagshipdemo_omnisim.txt.newton.json`. The analyzer used here is
`_scratch/g1_endurance/analyze.py` (scratch, read-only, not committed). Raw traces for all six runs
are in `_scratch/g1_endurance/run{1..6}*_rl.txt`.
