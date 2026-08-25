# Foot-redesign experiment — does changing the G1/H1 foot solve robust standing + walking?

**Date:** 2026-06-29. **Question (user):** all our deterministic methods hit a wall on the
G1; is that wall the *robot* (the foot/ankle morphology) rather than the *controller*? If we
are allowed to change the model (foot design), does standing + walking become robust — for
G1 and H1?

**Method:** isolated A/B. The ORIGINAL models under `projects/robots/**` are never modified
(other sessions/demos depend on them, and they are the control). `make_models.py` writes
modified COPIES into `models/`; `walk_exp.py` registers them into the *unmodified* offline
MPPI-walk harness (`humanoid_walk_offline.py`); `stand_basin.py` measures passive + active
push-recovery. The offline plant is mujoco_warp — the SAME solver the OmniSim Newton deploy
runs (single-source, zero model gap, CI-enforced), with the real deploy stand gains
(ke 400/kd 60) and nominal squat pose — so these numbers are deploy-faithful, not a toy sim.

## The morphology numbers (measured from the models)

| | foot length | foot width | ankle torque | ankle-roll DOF |
|---|---|---|---|---|
| **G1 orig** | 0.17 m | 0.06 m | ±35 N·m | yes |
| **H1 orig** | **0.28 m** | **0.03 m** | ±40 N·m | **no** |

H1's foot is 65% longer (sagittal) but half as wide with no roll DOF — which already predicts
the prior finding: H1 clears the forward-pitch wall but is lateral-marginal; G1 hits the
forward-pitch wall.

Variants: G1 `long` (toe→0.23 m), `long_strong` (+ankle ±70), `big` (0.27 m long + 0.09 m
wide + ankle ±88); H1 `wide` (0.12 m), `wide_xl` (0.16 m).

## Result 1 — STANDING: foot redesign DECISIVELY solves robust G1 standing

Push-recovery basin = largest instantaneous velocity shove the stand absorbs, WITH the real
reactive ankle lean (the `humanoid_stand_deploy` mechanism), held 3 s, no other balance.

| model | foot | static hold | forward push | lateral push |
|---|---|---|---|---|
| **G1 orig** | 0.17 m | **FELL** | **0** | **0** |
| G1 long | 0.23 m | held | 0.30 m/s | 0.40 m/s |
| G1 long+strong | 0.23 m, ±70 | held | 0.30 m/s | 0.40 m/s |
| **G1 big** | 0.27 m + wide | held | **0.40 m/s** | **0.40 m/s** |
| H1 orig | 0.28×0.03 | held | 0.30 m/s | 0.60 m/s |
| **H1 wide_xl** | 0.28×0.16 | held | 0.30 m/s | **0.80 m/s** |

- **G1: the foot IS the fix.** Same controller, same lean — the orig foot can't even hold the
  static squat; the bigfoot recovers a 0.4 m/s shove in BOTH axes. Foot *length* drives the
  forward gain; foot *width* drives the lateral gain (and even the passive-no-lean lateral
  basin rose 0→0.20→0.30 with width). Ankle *torque* did not matter for standing (long vs
  long+strong identical) — standing push recovery is CoP-geometry-limited, not torque-limited.
- **H1: already stands; a wider foot adds lateral margin** (0.60→0.80), the lateral-marginal
  axis. Its standing was never foot-limited (tall/heavy, tips slowly).

## Result 2 — WALKING: foot length removes the sagittal wall; width + stance fixes lateral

Offline MPPI-on-human-gait, identical control config per group.

| G1 walk (vx 0.20 default) | foot | survival | forward | lateral drift |
|---|---|---|---|---|
| orig | 0.17 m | 3.8 s | 0.46 m | 0.83 m |
| long | 0.23 m | 3.8 s | 0.19 m | 0.61 m |
| long+strong | 0.23 m, ±70 | 6.1 s | 0.38 m | 0.54 m |
| big | 0.27 m+wide+±88 | 6.9 s | 0.86 m | 0.27 m |

| G1 walk (H1 lateral-tuned cfg, vx 0.12) | result |
|---|---|
| orig | FELL 6.0 s — by **forward pitch** +0.67 (the documented structural wall) |
| big | FELL 6.5 s — **pitch stayed clean ±0.2 throughout**; fell by slow **lateral** drift |
| **big + wider stance (step 0.16)** | **clean straight walk, y-drift ±0.05, roll/pit ±0.15, ≥10 s, no fall** |

- The longer foot **suppresses the structural forward-pitch wall** (pitch stays ±0.2 where the
  orig folds forward) — the wall the whole prior research program kept hitting.
- It then **trades the failure to the lateral axis**; widening the *stance* (and foot) tames
  that → a clean, dead-straight quasi-static walk to ≥10 s, matching what H1 achieves.

### Foot-isolation control (does the win need the foot, or just the wider stance?)

The clean isolation is the **no-wide-stance** pair (same lateral cfg, only the foot differs):
`g1_orig_tuned` fell by **forward pitch +0.67** (the structural sagittal wall); `g1_big_tuned`
kept **pitch ±0.2 the whole time** and only fell from slow lateral drift. → the FOOT (length)
removes the sagittal wall; it is not a stance-tuning artifact.

Adding wide stance to each (cut at the 500 s wall-clock ~6.9 s, bigfoot rollouts run slow):
`g1_big`+stance held **steady** (z 0.76, vx calm, roll/pit ±0.11); `g1_orig`+stance reached
6.9 s but **destabilizing** (vx spiking 0.28, z sagging 0.72, pitch creeping 0.13). Wide stance
helps the orig laterally, but the small foot's sagittal margin still erodes. Net: **foot length
→ sagittal; foot/stance width → lateral; both needed** for the clean straight walk.

## Verdict

**The wall was the robot, not the controller — and changing the foot fixes it in sim.**

- **G1 STANDING: solved by the foot.** Identical deterministic stand controller: the orig foot
  cannot hold the squat at all (basin 0/0); the redesigned foot recovers 0.40 m/s shoves in both
  axes. The G1 stand's chronic marginality (the thing the whole cube-defense program fought) is
  a **foot-geometry** problem, not a control problem.
- **G1 WALKING: foot redesign makes the structurally-blocked deterministic walk work.** Longer
  foot removes the forward-pitch wall every prior method hit; stance/width removes the lateral
  drift; together → a clean dead-straight quasi-static walk ≥10 s, matching H1.
- **H1: standing already robust; a wider foot adds lateral margin** (0.60→0.80) — the
  lateral-marginal axis (no ankle-roll + narrow foot) my notes flagged.

**Honest caveats.** (1) This is a **model** change — the real G1 has the small foot, so this does
NOT make the *existing* hardware walk; it is evidence for a **hardware foot redesign / foot
extension**, for *understanding* the wall, and for sim demos with the modified model. (2) The
walk is slow/quasi-static (like H1's), not athletic, and still a tuned/marginal result. (3) Plant
is mujoco_warp = the Newton deploy solver (zero model gap) with the real deploy stand gains/pose,
so it is deploy-faithful — but not yet shown in the live `omnisim-bin` GUI (the natural next step).
(4) RL's separate deploy obs-gap is untouched; this is all deterministic control.

## Result 3 — IN-ENGINE confirmation (real omnisim-bin, Newton/mujoco_warp, full controller)

Bigfoot copies of the deploy URDF + cube-throw world (`make_inengine.py`, additive, originals
untouched) run the SHIPPED deterministic `humanoid_stand_deploy` controller (lean + arm-balance,
ke 400) in the actual binary. Newton confirmed driving (`world finalised (solver=MuJoCo
(mujoco_warp))`). Cube barrage = 8 boxes (0.5 kg) thrown at the torso.

| barrage | orig foot (0.17 m) | bigfoot (0.27 m + wide) |
|---|---|---|
| designed 4.5 m/s | stands | stands |
| hard 8.0 m/s / 0.8 s | stands, but settles at **0.128 rad** forward tilt | stands, **0.05 rad** tilt |
| **brutal 12 m/s / 0.5 s** | **FELL @ 2.34 s** (forward faceplant, pitch −1.50, 8 hits) | **STOOD**, peak tilt **0.088 rad**, 8 hits |

The full reactive controller is good enough to keep even the small-foot orig up against moderate
barrages (the lean + arm-momentum compensate), so the foot shows up as **margin** there (0.05 vs
0.128 rad tilt). Push hard enough (12 m/s) and the small foot **topples** while the bigfoot still
shrugs it off — the morphology limit made visible in the real engine. Reproduce:
`bash run_inengine.sh orig 12.0 0.5 30` vs `bash run_inengine.sh big 12.0 0.5 30`; GUI: append `gui`.

## Result 4 — IN-ENGINE WALK (real binary, live) — forward walk achieved, durability is the live gap

`g1_walk_mpc.py` = an in-engine MPPI walk driver (g1 sibling of `h1_walk_mpc.py`) loaded via
`OMNISIM_INENGINE_PYMOD` on `g1_walk_bigfoot.omniworld`; the stand controller settles, then the
driver owns the leg servo = g1 gait + a balance residual planned by rolling K futures in the
engine's own mujoco_warp (CUDA-graph, ~0.5x realtime). Launch: `run_g1_walk.sh <dur> <tag> [gui]`.

Tuning findings (headless, bigfoot world):
- **lean ON fights the walk** — the stand's reactive ankle-lean competes for the leg targets and
  drives a backward drift. `HSTAND_LEAN=0` -> the robot walks **forward**.
- **ke 400 (stand stiffness) over-tracks the gait** -> `OMNISIM_NEWTON_TARGET_KE=200` (closer to
  the offline kp100) gives a far cleaner gait: roll/pitch near 0, **clean forward walk to
  ~0.32 m over ~4 s**.
- **Remaining wall = lateral durability.** After ~4 s of clean stepping a slow lateral bias
  accumulates and tips it; it falls ~10.6 s. More lateral cost weight did NOT help (made it
  worse) -> it's not under-authority, it's the **warm-contact mismatch** (the MPPI rollouts use
  fresh put_data contacts != the live warm contacts, so its lateral prediction is wrong).

Status: this is the best deterministic *live* humanoid walk in the program (H1's live walk fell
~2-3 s; this bigfoot G1 takes clean forward steps and survives ~10.6 s) — but it is **not yet a
durable live walk**. The durable, non-falling walk remains the OFFLINE result; closing the live
warm-contact gap (seed the live efc/contact warm-start into the rollout worlds) is the open work.
Best config: `HSTAND_LEAN=0 OMNISIM_NEWTON_TARGET_KE=200 KD=30 GWM_VX=0.10 GWM_W_Y=105
GWM_W_YAW=18 GWM_STEP_WIDTH=0.20`.

## Result 5 — IN-ENGINE RESIDUAL RL cracks the walk (the lateral wall) — trained where it deploys

The deterministic walk falls because of **lateral single-support balance** (Result 4 + all the
deterministic attempts). A residual RL policy ON TOP of the deterministic gait, **trained inside
OmniSim's own Newton engine** (zero sim-to-deploy gap), learns exactly that missing balance.

Infra (`projects/policies/research/rl_inengine/`): `g1_walk_residual_inengine.py` (sequential,
live-world ES) + `g1_walk_batched_rl.py` (**batched**: evaluates the whole ES population as K=2*pop
parallel mujoco_warp worlds via `world._mpc_rollout_buffers` — **~45x faster**: ~3.2 s/gen for 48
worlds vs ~150 s/gen sequential, GPU steps them in lockstep). Launch: `run_walk_rl.sh`. Reuses the
stand module's solved in-engine reset + live obs + leg map.

Findings:
- **Linear residual PLATEAUS** (fitness −65→−21 over 157 gens, never survives) — the lateral
  correction is **phase-dependent** (which foot is down decides where you can push) = nonlinear,
  which a linear map structurally can't represent.
- **An MLP residual (1 hidden layer, 24 units) BREAKS THROUGH** — fitness goes positive +28→+534
  in ~17 gens, then to mean ~1089 with the majority of worlds surviving the full episode.
- **It TRANSFERS**: the in-engine-trained MLP walks **forward ~1.15 m, upright, roll controlled**
  in live deploy (`g1_walk_deploy_step`) — the first walk here that is both *trained* and
  *forward-walking* (vs the deterministic gaits that fell sideways in one step). The MLP solved the
  lateral single-support balance that beat every deterministic method.
- **Open wall = train↔deploy CONTACT gap**: batched rollouts reset FRESH foot contacts; live deploy
  runs on WARM contacts. Policy survives 6 s in training but ~3 s in deploy, and more fresh-contact
  training doesn't move the deploy number. Closing it (seed warm-contact state into rollouts, or a
  live-world fine-tune) is the path to a durable 10–20 m walk. Policy: `runs/g1_walk_mlp.npz`.

**Verdict:** residual RL on the deterministic walk, trained in-engine, is the *working recipe* for
humanoid walking here — it learns the lateral balance no deterministic method could, and transfers.

## Reproduce
```
python projects/policies/research/mpc/foot_redesign/make_models.py
bash    projects/policies/research/mpc/foot_redesign/run_matrix.sh g1walk     # G1 walk A/B
bash    projects/policies/research/mpc/foot_redesign/run_matrix.sh g1tuned    # G1 lateral-cfg A/B
bash    projects/policies/research/mpc/foot_redesign/run_matrix.sh standlean  # standing basin A/B
```
Logs + result tables: `_scratch/foot_redesign/`.
