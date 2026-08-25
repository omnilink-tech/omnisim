# OmniQuad motions via the shadowing pipeline

Three OmniQuad motions, each produced the same way — a **feasible
"ghost" reference** + an **RL policy that shadows it** ("planning describes, control
solves") — and each **deployed in OmniSim Newton** with a translucent ghost beside
the real robot. This is the shadowing pipeline (`projects/policies/research/shadowing/`,
[shadowing.md](shadowing.md)) applied to OmniQuad; the per-robot tracker lives in
`projects/policies/research/training`.

| Motion | Ghost source (Component 1) | Tracker (Component 3) | Deploy | Launch |
|---|---|---|---|---|
| **Walk / stop / walk** | analytic foot-space trot model (`omniquad_trot_gait.py`) | velocity-conditioned residual (`gpu_omniquad_walk_vc_main`) | RL the whole time | `run_omniquad_walk_vc_ghost_demo.ps1` |
| **Crouch–recover** | MPPI `ghost_generator` | none — **feedforward** | replays the ghost open-loop | `run_omniquad_crouch_deploy.ps1` |
| **Get-up from the ground** | MPPI `ghost_generator` | reference-tracking residual + heavy DR (`gpu_omniquad_getup_main`) | RL rise → faded hold | `run_omniquad_getup_ghost_demo.ps1` |

All three: OmniSim Newton, `OMNISIM_NEWTON_TARGET_KE=500 KD=60 SUBSTEPS=8 MJWARP=1
GROUND_MU=2.0 URDF_USE_INERTIA=1` (the deploy drive the trot/ghost references were
built against), **zero falls**.

## The pipeline, concretely for OmniQuad

```
 (A) intent ──► (B) ghost (feasible by construction) ──► (C) RL shadows it ──► (D) deploy in OmniSim
                  trot model  OR  MPPI ghost_generator      residual on the ref    real robot + ghost
                  + ghost_verifier certificate (Component 2)
```

- **Component 1 — the ghost.** For a *gait*, the analytic trot model is the feasible
  generator (stance slides at −vx, quintic swing; feasible by construction). For
  *non-gait* motions (crouch, get-up) the **MPPI receding-horizon `ghost_generator`**
  discovers a feasible trajectory over OmniQuad's real dynamics. OmniQuad's dump has
  anonymous, interleaved (pos+vel) actuators and no foot names, so the generators
  **override the GhostGenerator mappings on the instance** (pos actuators at even ctrl
  idx; foot bodies 4/7/10/13) — see `projects/policies/research/tools/generate_omniquad_{crouch,getup}.py`.
- **Component 2 — the verifier.** `ghost_verifier` re-sims the ghost: the **open-loop
  gate** (does it stay upright, even if it needs feedback?) decides; the
  inverse-dynamics certificate is experimental and over-reports (it fails for the
  accepted G1 ghost too). `projects/policies/research/tools/verify_omniquad_walk_ghost.py` certified the
  walk ghost; the get-up generator runs the gate inline.
- **Component 3 — the tracker.** `gpu_mjwarp_omniquad_getup_trainer.py` (the walk trainer
  retargeted): `--track-ref <ghost.npz>`, **RSI** resets to sampled states from the
  ghost `q/qvel` (all feasible + on-distribution), obs = `[vlin, vang_BODY, proj_g,
  q−ref, qd, last_action, phase(b, b_ahead, z_err, pitch_err)]` (49), imitation reward
  (joint + base-height gaussians) + alive + upright. Trained via
  `gpu_mjwarp_omniquad_getup_trainer.py` (run locally).
- **Component D — deploy.** A mimic controller replays the certified ghost's position
  targets + the RL residual on the URDF OmniQuad in Newton.

## The deploy gotchas (what it took to actually land each)

1. **Velocity conditioning (walk/stop).** One policy, commandable forward speed incl.
   0 = stand (`OMNIQUAD_VX_CMD_MAX`); the gait scales toward the standing pose. Bias the
   speed sampling to the two milestone speeds (42% nominal / 33% zero / 25% uniform) —
   a flat sample dilutes the walk. Straightness needs `--wz-range` (learned yaw
   steering) so the deploy heading-hold can correct drift; chunks overfit/oscillate →
   **select by deploy** (the earliest clean chunk wins).
2. **Quasi-static deploys feedforward; dynamic ones need RL+DR.** The crouch is
   quasi-static and the ghost is feasible, so it deploys **open-loop, 0 falls**. The
   get-up is a contact handoff → feedforward sags → it needs the RL tracker.
3. **Match the deploy START to the trainer (teleport).** Newton's *free* settle can't
   form the belly-flat lying pose (it rests half-propped at 0.25/22° = OOD). The get-up
   controller is a Supervisor that **teleports its base+joints to the ghost's first
   frame** (`base_z=0.103`) + zeroes velocity → start matches the trainer RSI exactly.
4. **Heavy DR crosses the mujoco_warp→Newton gap.** The first get-up policy flipped on
   the rise; a DR retrain (mass .15 / fric .25 / kp .15 / tilt/vel/push) fixed it.
5. **Fade the residual at the end of a one-shot motion.** The get-up rose then
   *collapsed* the hold ~2 s later — the rise-trained residual over-actuates a now-
   static stand. Fix: once the reference ends, **fade the residual to 0 and ramp the
   baseline to the proven trot stand** (`OMNIQUAD_GETUP_HOLD_FADE_S`) → crisp upright stand
   (base_z 0.545, tilt <2°), holds indefinitely.

## The frontier: forward leap (where the pipeline starts to break)

A **forward leap** (real flight phase) was the deliberate limit-finder — it hits limits
#1 (verifier can't certify flight), #3 (tiny residual vs an explosive launch), #4
(landing/launch deploy gap) at once.

- **C1 works:** MPPI + a **launch-velocity task objective** (the toss mechanism, driving
  the *base's* own velocity) produced a real leap — peak base_z 0.92, **0.40 s with all
  four feet airborne**, +0.75 m, landed upright in the ghost. `generate_omniquad_jump.py`.
- **C2 limit:** the open-loop "doesn't fall" gate is meaningless (a leap *is* a fall) →
  used a jump-aware check (launched? flight? landed?).
- **C3 trains** (reuses the generic tracker, flight-tolerant z-weight, heavy DR).
- **Deploy — the wall, and how far we pushed it:**
  1. It first **tumbled** (over-rotated → landed inverted). Root cause was a deploy-start
     bug: the ghost's `ctrl[0]` is a *noisy MPPI control*, not a clean stand, so the
     teleport collapsed the start. **Fixed** by teleporting a standing-start ghost to the
     clean trot stand → no tumble, stable landing.
  2. **Chaotic ghost → fixed:** the raw MPPI leap was L/R-asymmetric (twisting; asym
     2.2 rad). `symmetric_generate` searches a **6-DOF L/R-mirrored basis** → asym
     0.0 *and a bigger leap* (peak 1.046, 0.54 s flight). The deploy is then clean +
     stable (symmetric, 0 falls, lands 0.55/tilt <2°).
  3. **Height ceiling (the deep wall):** the deploy **under-jumps** and *saturates* at
     **~0.69 m vs the 1.05 m ghost.** Pushed with three levers, all marginal:
     symmetric ghost (0.655) → more `--res-scale` 0.20→0.25 (no change) → a
     **launch-velocity reward** (`--rw-launchvel`, gpu_omniquad_jump_lv) → 0.693.
- **Diagnosis:** the ghost reaches 1.05 m with the *same* kp500 actuators in mujoco,
  so it's **not actuator power** — the launch **push-off impulse doesn't transfer
  through the mujoco_warp→Newton contact gap** (Newton's brief launch contact yields
  less). The deep fixes are bigger infra: **contact-stiffness (solref/solimp) DR**,
  Newton-native fine-tune, or a **torque-space launch action** (not a position
  residual). Also: a quadruped can't re-orient mid-air (leg inertia ≪ body), so the
  launch must be ~rotation-free — which the symmetric ghost gives.
  Policy `gpu_omniquad_jump_main`; `run_omniquad_jump_deploy.ps1` / `run_omniquad_jump_ghost_demo.ps1`.

**Net:** the pipeline cleanly deploys ground-contact motions (walk, crouch, get-up) and a
*clean, stable but damped* leap; the *full-height* leap is the deploy frontier, walled by
the launch-impulse contact gap rather than by stability or the ghost.

## Files

- Gait model: `projects/policies/control/gait/omniquad_trot_gait.py` (`speed_scale` = velocity conditioning).
- Generators (C1): `projects/policies/research/tools/generate_omniquad_crouch.py`, `generate_omniquad_getup.py`.
- Verifier helper (C2): `projects/policies/research/tools/verify_omniquad_walk_ghost.py`.
- Trainer (C3): `projects/policies/research/training/gpu_mjwarp_omniquad_getup_trainer.py` (`--track-ref`).
- Certified ghosts: `projects/policies/research/shadowing/ghosts/omniquad_{crouch,getup}_ghost.npz`.
- Deploy controllers/worlds: `projects/policies/controllers/omniquad_{walk_deploy,crouch_mimic,getup_mimic,ghost,getup_ghost}/`, `projects/policies/worlds/omniquad_*_deploy.wbt` + `*_ghost_demo.wbt`.
- Policies: `projects/policies/research/inference/policies/gpu_omniquad_walk_vc_main/`, `gpu_omniquad_getup_main/`.
- Launch scripts: `scripts/dev/run_omniquad_{walk_vc_ghost_demo,crouch_deploy,getup_deploy,getup_ghost_demo}.ps1`.

## Related
- [shadowing.md](shadowing.md) — the pipeline (generator / verifier / tracker).
- [ghost-tracking-pipeline.md](ghost-tracking-pipeline.md) — the "any robot, any motion" thesis.
- [omniquad-residual-rl.md](omniquad-residual-rl.md) — the earlier OmniQuad residual-RL walk work.
