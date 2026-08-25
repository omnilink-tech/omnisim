# humanoid_static_walk — deterministic quasi-static humanoid walk (no RL)

The walking sibling of [`humanoid_stand_deploy`](../../../controllers/humanoid_stand_deploy/). It
reuses that controller's **proven control system verbatim** — a stiff Newton
position PD (set globally by the launcher via `OMNISIM_NEWTON_TARGET_KE/_KD`),
driven only by `motor.setPosition(target)`, no RL, no per-motor PD — and the
same `HUMANOID_STAND_SPEC` JSON. The one addition is a slow, phase-clocked,
**statically-stable gait** layered on the squat nominal.

## Why quasi-static

The repo's deterministic-brain work
([docs/developer/g1-deterministic-brain.md](../../../../../docs/developer/g1-deterministic-brain.md))
proved a *dynamic* reactive walk caps at ~2.3 s: G1's fall timescale
√(z/g) ≈ 0.28 s is shorter than a step period, so a reactive step can't catch
the fall. A **quasi-static** gait sidesteps that: it is slow and keeps the CoM
over the stance foot at every instant, so the robot is always in a stand-like
statically-stable configuration — the property the deterministic stand already
guarantees. It is a careful shuffle, not an athletic stride, but it is
deterministic and robust. This is distinct from the `humanoid_walk_deploy`
sibling, which tracks a *dynamic* kinematic shadow (and is the one that caps at
~2 s).

## Gait

Each leg cycles SWING → STANCE:
- **SWING**: lift the foot (knee), sweep the hip from behind to ahead → places
  the foot a step forward. The other leg is in stance with the body leaned onto
  it (lateral weight-shift), so single support stays statically stable.
- **STANCE**: foot planted flat; the hip sweeps from ahead to behind as the
  pelvis advances over it. Propulsion is **stance-foot progression, not
  push-off** (the deterministic-brain finding). The ankle counter-rotates to
  keep the foot flat through the sweep.

Lateral lean is a smooth cosine phased so the body is over the stance leg
whenever the other leg swings. With all gait amplitudes 0 the controller reduces
**exactly** to the deterministic stand.

## Run

```powershell
# GUI (watch it walk)
powershell -File scripts/dev/run_humanoid_static_walk.ps1 -Robot g1 -Gui

# headless sweep (forward distance vs no-fall)
powershell -File scripts/dev/run_humanoid_static_walk.ps1 -Robot g1 -Duration 40 -Stride 0.18 -Lateral 0.12
```

## Tuning knobs (env `WALK_*`, or launcher `-Param`)

| knob | meaning |
|---|---|
| `WALK_PERIOD` | full-stride period, s (bigger = slower = more static) |
| `WALK_STRIDE` | hip-pitch swing amplitude, rad (forward step size) |
| `WALK_LIFT` | swing-leg knee lift, rad |
| `WALK_LATERAL` | lateral weight-shift (ankle-roll), rad |
| `WALK_HIP_ROLL` | hip-roll share of the lateral shift, rad |
| `WALK_FWD_LEAN` | constant forward CoM bias, rad |
| `WALK_DUTY` | swing fraction of each leg's cycle (rest is stance) |
| `WALK_LAT_LEAD` | phase lead of the lateral lean vs swing mid (cycles) |
| `WALK_LAT_SIGN` | ±1 sign of the lateral lean (tune once per robot) |
| `WALK_STRIDE_SIGN` | ±1 sign of forward progress (tune once per robot) |
| `WALK_RAMP_S` / `WALK_START_S` | ease the gait in / stand-still lead-in |
| `WALK_ANKLE_FLAT` | ankle counter-rotation fraction (flat foot) |
| `WALK_STOP_AT` | hold the stand after this forward distance, m (0 = off) |

`WALK_LAT_SIGN` and `WALK_STRIDE_SIGN` are the two signs to fix first per robot:
a headless run shows immediately whether it tips sideways (flip `LAT_SIGN`) or
walks backward (flip `STRIDE_SIGN`).
