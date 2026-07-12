# Residual RL for G1 push-recovery stepping — recipe + honest findings

RL as an **addition** to the deterministic capture-step work (not end-to-end): a small
PPO-trained residual on a deterministic gait+capture-point baseline, learning the
step coordination/timing the deterministic layer can't nail. Reuses the project's
canonical residual-RL trainer (`gpu_mjwarp_g1_walk_trainer.py`) + its matched deploy
controller (`g1_walk_deploy`) — configured for IN-PLACE PUSH RECOVERY instead of
forward walking.

## Recipe (validated, reproducible)
Train (local GPU ~310k env-steps/s, ~8 min, via `gpu_newton_g1_walk_repro_trainer.py`):
```
python projects/policies/research/training/gpu_mjwarp_g1_walk_trainer.py \
  --mjcf projects/robots/unitree/g1/urdf/g1_legs_kp100.mjcf.xml \   # MUST match deploy KE=100/KD=5
  --envs 4096 --iters 1500 --rollout 24 \
  --vx-target 0.0 --vel 0.3 --vel-sigma 0.20 \                      # in-place (no forward cmd)
  --gait-a-hip 0.10 --gait-a-knee 0.15 --gait-freq 1.3 \            # LOW gait amplitude (near-static)
  --cp-gain 0.8 --gait-a-lat 0.05 \                                 # capture-point + lateral weight-shift
  --dr-push-prob 0.02 --dr-push-vmax 1.0 \                          # random shoves = the learning signal
  --dr-init-q-band 0.05 --dr-init-xy-band 0.0 \
  --alive 2.0 --upright 1.0 --height -10.0 --term -15.0 \           # survive-don't-fall reward
  --res-scale 0.30 --save runs/g1_step_res/policy_kp100.pt
```
Eval (push survival): `--eval --eval-steps 512 --dr-push-prob 0.05 --dr-push-vmax <v>` + the same baseline flags.

## Result — SURPASSES the base in the trainer, hits the deploy gap in the engine
- **Trains cleanly**: reward dips on the pushes then climbs back; meanV 0 → **+119** over 1500 iters (converged). The residual learns push recovery on the baseline.
- **Trainer eval (the WIN)**: under a relentless barrage (a 1.0–1.5 m/s shove every ~0.3 s) it stays alive **99%** of steps (507/512), recovering **~6 consecutive 1.0 m/s** pushes before the first fall (~4–5 at 1.5 m/s). The deterministic base falls on a *single* 0.8 m/s push → **the residual clearly surpasses it in-distribution.**
- **Engine deploy (the WALL)**: the policy runs in OmniSim (Newton/mujoco_warp) but **topples at ~1.23 s even undisturbed**. `gerr≈0.28` (residual saturated at the 0.30 cap) → the policy sees OUT-OF-DISTRIBUTION obs in the engine (engine ang-vel ≠ trainer mujoco qvel; finite-diff qd; contact deltas), extrapolates, saturates, falls. This is the project's documented **train→deploy durability gap** (~1.2–1.8 s; only re-hosted Unitree policies clear it). See docs/developer/train-deploy-gap.md, closed-loop-chaos-diagnostic.md.

## Why this hit the gap, and the better architecture
The shortcut here put the RL residual on the walk trainer's **sine-gait baseline**, which
*does not balance on its own* — so the RL is doing the actual balancing, and that balancing
doesn't transfer (the OOD-obs saturation above). The **faithful** "addition to our work" is a
residual on the **deterministic STAND** (`humanoid_stand_deploy`), which already balances AND
deploys durably (122 s) — then the residual is a *small* correction the stable base carries
through the obs gap. That needs the deterministic stand ported into the trainer env (its squat
nominal + analytic ankle/lean balance as the batched baseline), which the walk trainer's
gait baseline isn't. That port is the next step if we pursue RL further.

## FAITHFUL attempt (residual on a "stable base") — and the wall it hit
Goal: put the residual on a baseline that DEPLOYS on its own, so the stable base carries
the residual through the obs gap. Retrained gait-OFF (static squat + ankle-PD), `--act -0.02`
(residual penalty → ~0 undisturbed), `--res-scale 0.5` (step-capable). Trainer converged
even better (meanV +136). Deploy: improved (fall 1.23 s → **2.34 s**) but still topples,
`gerr≈0.36` (residual scaled to 0.5 OVERRIDES the base on OOD engine obs).

**Decisive diagnostic — the walk baseline never deploy-stands.** Residual OFF (act-scale 0),
the gait-off baseline (nominal + ankle-PD) FALLS at **1.01 s @ kp100** and **0.75 s @ ke400**
(rolls over). So the `g1_walk_deploy` controller's baseline is NOT the deterministic stand —
it was built to be balanced BY the RL. Reusing the walk pipeline therefore CANNOT give the
"stable base carries the residual" property, at any stiffness.

**The actual deploy-durable stand is a DIFFERENT controller**: `humanoid_stand_deploy`
(ke=400 stiff hold + CoM-centering ank_bias=-0.06 + reactive lean + arm balance) stands 122 s.
The GPU walk trainer's `_baseline_targets_t` does not replicate that balance. So the truly
faithful "residual on the deterministic stand" needs:
  1. PORT the humanoid_stand baseline (squat nominal + ank_bias + reactive lean) into a
     kp400 trainer env (replace/augment the walk trainer's gait baseline), so the trainer-G1
     stands on the SAME baseline the deploy uses, and
  2. modify `humanoid_stand_deploy` to load the ONNX residual and ADD it to its targets.
That is substantial new infrastructure (a kp400 stand-baseline trainer), not a reconfig of
the walk trainer — and the residual would still face the obs gap (small + penalized residual
on a genuinely-stable base is the best shot, but unproven). This is the open next step.

**NET (honest):** RL-as-addition learns push recovery and SURPASSES the deterministic base
*in the trainer*, but does not deploy durably via the walk pipeline (deploy gap + the walk
baseline isn't a stand). A deployable RL surpass needs the stand-baseline port above — the
same train→deploy durability wall the project documents (only re-hosted Unitree policies clear it).

## Fixed along the way
- The trainer default `g1_legs.mjcf.xml` is **kp=20**; the engine deploys **kp=100** — a 5× stiffness mismatch (the historic deploy-fall cause). Train on `g1_legs_kp100.mjcf.xml`.
- `projects/policies/research/worlds/g1_walk_deploy.wbt` URDF path was broken by the rl→policies archival (`../../robots` → needs `../../../robots`). Fixed.
