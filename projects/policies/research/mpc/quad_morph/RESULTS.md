# Quad foot-redesign — does a contact-patch foot let DETERMINISTIC control walk a quadruped across terrain?

The quadruped analog of the G1/H1 foot-redesign (`../foot_redesign/`). Same discipline:
**isolated copies, the original deploy models are NEVER touched**; the validated offline
quad MPC harness (`../quad_mpc_offline.py`) is reused unmodified via a monkeypatch wrapper
(`walk_exp.py`). Plant = `mujoco_warp` = the Newton deploy solver → deploy-faithful, not a toy.

## The hypothesis

Every quad foot in the repo is a single **point-contact sphere** (omniquad r=0.035, go2 0.022,
b2 0.032). A point foot has **zero contact patch**, so:
1. it **slips** — the trot's stance foot is supposed to grip and translate the body
   backward-relative at −vx, but a point foot skates (the measured "~90% foot slip, marches
   in place" for the bare trot, and why the RL residual had to supply *all* the propulsion);
2. it gives **no per-foot yaw/roll resistance** — on rough terrain an asymmetric bump strike
   kicks the body and the point feet can't arrest it.

This is the same shape as the humanoid finding (a small foot caps the forward CoP moment).
Give each foot a real **contact patch** (a flat box sole, wide in the lateral axis) and the
deterministic MPC should gain the traction + heading authority to keep going **forward**.

The box bottom is placed at exactly the original sphere's bottom (sphere `pos_z − r`), and the
leg IK targets the kinematic tip independent of the geom, so the standing settle and the gait
are byte-for-byte unchanged. **Only the contact geom changes** — clean foot-isolation.

## Result — OmniQuad, deterministic MPC, `mujoco_warp` (8 s, K=96, H=24)

| foot | terrain | heading | **fwd speed** | y-drift | status |
|---|---|---|---|---|---|
| orig point-sphere | flat  | off | 0.230 m/s | +0.14 m | upright 8 s |
| box square patch  | flat  | off | 0.344 m/s | +0.51 m | upright 8 s |
| **box WIDE patch**| flat  | off | **0.427 m/s** | +0.40 m | upright 8 s |
| orig point-sphere | rough | off | 0.267 m/s | −0.04 m | upright 8 s |
| **box WIDE patch**| rough | off | **0.373 m/s** | −0.07 m | upright 8 s |
| orig point-sphere | rough | **on** (yaw6/wz2) | 0.255 m/s | −0.17 m | upright 8 s |
| **box WIDE patch**| flat  | **on** | **0.437 m/s** | −0.13 m | upright 8 s |
| **box WIDE patch**| rough | **on** | **0.472 m/s** | −0.17 m | upright 8 s |

**Findings:**
- **The contact patch ~DOUBLES forward speed** (flat 0.230 → 0.427 m/s, +86%; rough 0.267 →
  0.373 m/s, +40%). Same controller, same gait, same terrain — only the foot geom differs.
  The point foot slips and never reaches the 0.40 m/s gait target; the patch foot grips and
  hits/exceeds it. This is the quad analog of the G1 foot result: **morphology, not control.**
- **It's pure traction, not a terrain trick** — the win is just as large on FLAT ground.
- **The speed win is the FOOT, not the heading term**: orig + heading is still slow (0.255),
  box + heading is fast (0.472). The heading term only *straightens* — and it now works
  *because* the patch foot is stable enough to tolerate it (it destabilised the point foot,
  hence stock `YAW_W[omniquad]=(0,0)`).
- **Wide > square**: lateral width is the better patch axis (mirrors humanoid width→lateral).

## Why the terrain DEMO "shifts sideways" — and the two-part fix

The terrain demo (`omniquad_rough_track.omniworld`, `run_quad_rough_track.ps1`) runs the **RL residual
policy**, whose terrain finetune drifts ~40° sideways (documented). Two independent things fix it:
1. **Deterministic MPC instead of the RL policy** — the MPC's cost actively penalises lateral
   drift + (now) yaw, so it holds heading to a few degrees (y-drift ≤ 0.17 m over ~3.6 m here),
   versus the RL policy's 40°. This alone removes the "sideways" failure.
2. **The contact-patch foot** — fixes the slip that left the deterministic trot crawling, so it
   actually traverses *forward* at target speed (the part the demo also lacked).

Together: **wide-box foot + deterministic loco MPC + heading term** → forward at ~0.45 m/s,
straight, upright.

## Reproduce

```bash
python projects/policies/research/mpc/quad_morph/make_models.py omniquad go2 b2   # isolated model copies
# offline A/B (faithful mujoco_warp):
python projects/policies/research/mpc/quad_morph/walk_exp.py --base omniquad --foot orig    --terrain rough --secs 8 --yaw 6 --wz 2
python projects/policies/research/mpc/quad_morph/walk_exp.py --base omniquad --foot boxwide --terrain rough --secs 8 --yaw 6 --wz 2
```

## In-engine demo (real `omnisim-bin`, deterministic loco MPC)

`make_inengine.py` writes additive copies (originals untouched):
- `projects/robots/omnisim/omniquad/urdf/omniquad_bigfoot.urdf` — 4 foot spheres → wide box soles
- `projects/policies/worlds/omniquad_terrain_mpc_bigfoot.omniworld` — the rough-track world on that URDF

Driven by the in-engine locomotion MPC (`OMNISIM_INENGINE_MPC_LOCO` → `../quad_mpc_engine.py`,
no rebuild). Launcher (the world MUST stay in `research/worlds/` so the `omniquad_walk_deploy`
controller, in `research/controllers/`, resolves — a world in `policies/worlds/` falls back to
the `<generic>` controller and the gait never runs):

```bash
powershell -File scripts/dev/run_quad_mpc_engine.ps1 -Duration 60          # bigfoot + deterministic MPC
powershell -File scripts/dev/run_quad_mpc_engine.ps1 -Duration 60 -Orig    # A/B: stock point foot
powershell -File scripts/dev/run_quad_mpc_engine.ps1 -Duration 120 -Gui    # watch it
```

### In-engine confirmation (real `omnisim-bin`, Newton/`mujoco_warp`, headless)

- **Loco MPC engages** (`[inengine-mpc] loco maps OK ... gait=omniquad_trot_gait`) and the bigfoot
  OmniQuad walks **FORWARD at ~0.4 m/s, heading roughly held** — crossing the **3, 5, 7 cm** bumps to
  **x ≈ 9–10 m**. The "shifts sideways" demo failure (the RL policy's ~40° drift) is fixed: it goes
  forward, not sideways. Matches the offline speed — offline `mujoco_warp` IS the in-engine solver.
- **It does NOT complete the staircase track.** At every K (32/64/96) it reaches x ≈ 9–10 m then
  **face-plants at the 10 cm bump** (body pitches forward 60–70°, stalls nose-down or flips) and
  never reaches the 14/18 cm bars or the rubble field (x = 18–24). **Root cause: the trot's fixed
  foot clearance** (`step_height = 6 cm`) is shorter than the 10/14/18 cm bumps → the swing foot
  catches the bump's front face. The morphology fixed *slip, heading, speed*; it did **not** raise
  foot clearance, and the MPC is a residual on a fixed-clearance trot, not a foothold planner — so
  tall obstacles still stop it (exactly the previously measured terrain limit: clears
  ≈ foot-clearance then trips). Completing the full 18 cm + rubble track is the terrain-curriculum
  RL result, not this deterministic stack. An honest
  "walks the whole length forward" demo wants gentle/continuous (≤ ~6 cm) terrain or flat.
- **Bare gait (no MPC, no RL) drifts BACKWARD even on bigfoot** (x = −9 m over 131 s, rock-steady
  bz 0.58); a forward stride bias (`x0`) can't fix it (small = still backward, large = flips). The
  **propulsion comes through the closed-loop MPC residual** (distributed over all 12 joints), and
  the foot ~doubles *that* — the morphology win is real but only visible *with* the controller.
- **Speed — CUDA graph (this session):** the loco MPC now captures the `trot(phase+h)+δ` rollout
  into a CUDA graph via a per-step warp kernel that writes `ctrl = gait_buf[h] + δ` from device
  buffers (`_ctrl_kernel` in `../quad_mpc_engine.py`; the documented Route B, no rebuild). Plan
  time **~1.5 s → ~30 ms**; headless **0.04× → ~0.27×**; K=96 (offline quality) is now ~free.
  Remaining ceiling: the GUI's per-frame `mujoco_warp` readback caps any controller at ~0.4×, and
  the MPC's per-tick host↔device transfers keep the live demo ~0.1× (→ ~0.4× needs on-device
  scoring/apply). `MPC_LOCO_NOGRAPH=1` forces the old loop; `MPC_LOCO_ROLL_SUB` trims rollout substeps.

## Caveats
- **MODEL change**, not the real hardware foot — evidence for a foot/boot redesign + an
  explanation of the wall + a sim demo; it does not make the existing point-foot HW faster.
- Deploy-faithful (same `mujoco_warp` solver), wired in-engine, and the rollout is now CUDA-graph
  captured (~30 ms/plan). The live GUI runs ~0.1× and is render-bound at ~0.4×; not yet true 1×.
- **Honest scope:** this fixes forward propulsion, heading, and speed — NOT obstacle clearance.
  The deterministic stack does not complete the 18 cm staircase; it face-plants at the 10 cm bump.
- GPU rollout is mildly nondeterministic run-to-run; the ~+85% gap is far outside that noise.
- **Generalizes across the whole fleet** (offline flat, deterministic MPC, same point-sphere →
  wide-box-patch swap, same controller): OmniQuad 0.230 → 0.427 m/s (**+86%**), Go2 0.271 → 0.347
  (**+28%**), B2 0.536 → 0.681 (**+27%**). B2's patch foot drifts more laterally (−2.24 m) because
  B2 runs with no heading term by default — the same "faster propulsion wants heading control"
  pattern OmniQuad showed; add the yaw term (as OmniQuad) to tighten it. (No go2/b2 rough MJCF yet, so the
  terrain A/B + the in-engine demo are OmniQuad-only.)
