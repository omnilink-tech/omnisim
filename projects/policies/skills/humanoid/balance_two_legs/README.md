# Skill: balance_two_legs — hold a two-leg stand and reject external pushes

**Class:** humanoid · **Robots:** G1, H1 · **Kind:** deterministic · **Status:** ✅ verified

The deterministic two-leg stand and the **balance core** every other humanoid posture/arm
skill rides on top of. A stiff, statically-stable squat held by a Newton position PD, kept
centred by a reactive ankle lean, a fast arm-swing balancer, and a hip return-to-home
integral. The reference demo (`-Throw`) hurls cubes at the torso one at a time from every
side — the robot keeps standing (the cube-defense demo).

## Run

```powershell
python projects/policies/skills/run_skill.py balance_two_legs --throw          # cube-defense
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -ArmsDown -Throw -Gui
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1 -Duration 30
```

## How it balances

- **Stiff squat hold** — every joint position-held at a statically-stable nominal squat
  (`ank_bias` centres the CoM over the feet).
- **Reactive ankle lean** — leans the ankles back the instant a push moves the body forward
  (disturbance-driven, catches a cube from behind before it face-plants).
- **arm_balance** — the arms are the *fast* balancer: shoulders swing to shift the CoM and
  counter-rotate (kp=6). This is why G1's marginal stand holds — and why the
  [`g1_arm_motion`](../g1_arm_motion/) skill must keep this live rather than commandeering the arms.
- **hip auto_trim** — a slow integral that returns the body all the way to 0 rad upright.

## Verified

G1 stands robustly (0 falls over 400 s+ sim), holds all thrown cubes and returns pitch &
roll to ~0; runs robust **cold**. H1 stands via the same generic harness with
per-robot specs. Canonical status: [docs/developer/rl-current-state.md](../../../../../docs/developer/rl-current-state.md),
[docs/developer/humanoid-deterministic-stand.md](../../../../../docs/developer/humanoid-deterministic-stand.md).

## This is the core other skills overlay

`humanoid_stand_deploy` is a **balance core + composable overlays**. `balance_two_legs` is
that core. Overlays that ride on it: [`g1_arm_motion`](../g1_arm_motion/) (3-D arm exercises),
`one_leg`, `manip` (pick/place), `squat`. Each declares which effectors it owns; the core
keeps the robot upright underneath. See [../../README.md](../../README.md) for the contract.
