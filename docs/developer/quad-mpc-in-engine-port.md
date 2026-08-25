# In-engine quad-locomotion MPC — ready-to-execute port plan

**Status (2026-06-28):** the control design is **validated offline** across all three
quads; the in-engine port is **specced here, not yet applied**. This doc is the
clean drop-in plan for wiring it into the engine MPC once
`src/omnisim/physics/OmNewtonBackend.cpp` is free of concurrent edits.

Intent: transform all quadruped robots to use MPC (walking), with the new
quad code **isolated** so it doesn't collide with the concurrent session's in-engine
*stand* MPC.

---

## 1. What's already proven (the spec the port implements)

Offline prototype: [`projects/policies/research/mpc/quad_mpc_offline.py`](../../projects/policies/research/mpc/quad_mpc_offline.py)
(commits `7b68956a` → `8678d233`). Same `mujoco_warp` solver + deploy MJCF the engine
uses, so it faithfully proxies in-engine behaviour.

**Design — the deterministic analog of the RL residual, PLANNED not learned, zero model-gap:**
- **Nominal** = the trot gait targets at the current phase, **advancing over the
  horizon** (`trot(phase + h)` for rollout step `h`). The deploy controller already
  writes the trot onto `control.joint_target_pos`; the MPC residual rides on top.
- **MPPI** samples a residual `δ` (constant over the horizon) on the **12 leg joints**,
  rolls `K` copies `H` control-ticks forward (gait advancing), scores each by a
  **locomotion cost**, applies the softmax-weighted `δ`. Receding horizon.
- **No training, no H100** — it's a planner.

**Locomotion cost** (per rollout, final state; weights from the prototype):
```
J = W_VX*(vx - vx_target)^2          # track target speed (NOT raw distance -> no lunge)
  + W_UP*(roll^2 + pitch^2)
  + W_RATE*(rollrate^2 + pitchrate^2) # angular-rate damping -> durability
  + W_YAW*yaw^2 + W_WZ*wz^2           # heading (PER-ROBOT, see below)
  + W_Y*(y - y0)^2                    # lateral drift
  + W_H*max(0, zref - z)^2            # hold height (weight HARD; a small sag is ~free)
  + W_VZ*max(0, -vz)^2               # penalise downward velocity (body sinks else)
  + W_RES*|δ|^2
  + FALL_PEN if (z<floor | |roll|>0.8 | |pitch|>0.8)
weights: W_VX 10, W_UP 8, W_RATE 3, W_Y 6, W_H 120, W_VZ 40, W_RES 0.2, FALL 300
zref = body_height - 0.02 ; vx_target = gait vx
PER-ROBOT heading (YAW_W): go2 (12,4) -> dead straight ; omniquad/b2 (0,0) -> straighter
without it (the yaw residual perturbs the heavier robots' balance)
```

**Tuning lessons baked in (don't relearn):**
- Track a **target speed**, never reward raw distance → else it lunges to 1.2 m/s and
  topples at 0.9 s.
- Weight **height hard + penalise downward vz** → else the body sinks then collapses.
- **Angular-rate damping** is what bought durability (OmniQuad 8 s fall → 14 s upright).
- Heading weights are **per-robot** (small/light go2 wants strong yaw; heavier
  omniquad/b2 don't).

**Offline results** (mujoco_warp, ~0.4 m/s target): OmniQuad 14 s upright @ ~0.27 m/s
(H=24 + rate damping); Go2 & B2 10 s+ upright, dead straight, ~0.3 m/s. Run-to-run
variance from GPU rollout nondeterminism.

**Joint selection (robot-agnostic):** 12 leg hinges in controller order
FL,FR,RL,RR × (hip_x, hip_y, knee). Classify by axis + range:
`hip_x` = the only X-axis hinge (`|axis.x|>0.5`); `knee` = the Y-hinge whose **upper**
bound is negative (`jnt_range[1] < 0`, true for omniquad/go2/b2); else `hip_y`. Map to
Newton DOF via the solver's `mjc_jnt_to_newton_dof` (authoritative — SolverMuJoCo
renames joints "joint_<N>").

---

## 2. The blocker (why it's not a 10-line hook)

The engine stand-MPC is real-time only because `_mpc_plan` captures the `H*sub`-step
rollout into a **CUDA graph** (~30 ms/plan; ~50× slower without it → ~1.5 s, which
stalls the 16 ms live loop). The graph works for the stand because the control target
is **constant** over the horizon.

Locomotion needs `ctrl = trot(phase+h) + δ` — the target **changes every horizon
step** — which a single captured graph can't express. This is the one real piece of
new engine work.

---

## 3. Two routes

### Route A — headless-first, no graph (lower risk; do this first)
Validate zero-gap in-engine walking before optimising. In `_mpc_plan_loco`, drop the
graph and run the plain step loop, updating `ctrl` each horizon step from a
precomputed `gait[h]` buffer + `δ`. Slow (~1.5 s/plan) but fine for a **non-real-time
headless** run (`run_quad_rough_track.ps1` runs as-fast-as-possible, not wall-clock).
Confirms the engine reproduces the offline numbers. ~1 rebuild.

### Route B — real-time, gait-aware CUDA graph (the proper fix)
Precompute `gait_buf` (shape `H × NR`, the trot targets at `phase+h` for the residual
joints, host→device once per plan). Write a **warp kernel** `set_ctrl_step(h)` that
does `rd.ctrl[world, act[j]] = gait_buf[h, j] + delta[world, j]` for each residual
joint `j`. Capture the rollout as `for h in H: set_ctrl_step(h); for s in sub: step`.
Re-launch each plan after assigning fresh `gait_buf` + `delta`. Restores ~30 ms/plan.
`δ` stays constant over the horizon (per-world), so only `gait_buf` + `delta` change
between launches — graph-compatible.

---

## 4. Exact additive patch points in `OmNewtonBackend.cpp`

All additive + env-gated → the existing G1-stand path is untouched (no conflict with
the concurrent session's work) and the default build is unchanged. The MPC lives in
the embedded-Python block; a Python bug surfaces only at runtime (with the env var
set) and is caught by the existing `try/except` + `_mpc_log`, so it **cannot break the
C++ build** as long as the string literal stays well-formed.

Add three sibling methods next to the existing ones (reuse `_mpc_rollout_buffers`,
`_mpc_seed_qv`):

1. **`_mpc_loco_maps(self)`** — like `_mpc_build_maps` but selects the 12 leg joints
   (classifier in §1) and sets per-joint `sigma`. Caches `self._loco_act`,
   `self._loco_dof`, `self._loco_ready`.
2. **`_mpc_plan_loco(self, K, H)`** — Route A or B rollout above; cost from §1; updates
   `self._loco_nom` (the chosen residual). Reads weights from `MPC_LOCO_*` env (so they
   stay per-robot without recompiling). Needs the trot: either import
   `projects.policies.control.gait.<robot>_trot_gait` (add repo root to `sys.path` in
   the hook) and a gait clock `phase += 2π*freq*dt` synced to the controller
   (controller starts at `QS_PHASE`), or read `control.joint_target_pos` as `gait[0]`
   and advance analytically.
3. **`mpc_loco_step(self)`** — entry: build maps once, maintain the gait clock, call
   `_mpc_plan_loco` every `MPC_REPLAN_EVERY`, apply `self._loco_nom` onto
   `control.joint_target_pos` at the residual joints' Newton DOFs (mirror the apply
   block at the end of `mpc_stand_step`).

**Call site:** wherever `mpc_stand_step` is invoked under `OMNISIM_INENGINE_MPC`, add a
parallel branch:
```python
if os.environ.get("OMNISIM_INENGINE_MPC_LOCO"):
    self.mpc_loco_step()
elif os.environ.get("OMNISIM_INENGINE_MPC"):
    self.mpc_stand_step()
```

**New env vars (all default-off / sensible):** `OMNISIM_INENGINE_MPC_LOCO`,
`MPC_LOCO_VX`, `MPC_LOCO_FREQ`, `MPC_LOCO_K`, `MPC_LOCO_H`, `MPC_LOCO_GAIT_MODULE`,
and the `MPC_LOCO_W*` cost weights (default to §1).

---

## 5. Coordination & validation
- **Coordinate** off `OmNewtonBackend.cpp` with the concurrent session before editing;
  it's their hot file. Then `build_omni.bat` (one .cpp → incremental relink).
- **Validate:** run a quad on a flat then the rough-track world with
  `OMNISIM_INENGINE_MPC_LOCO=1` (bare trot controller, residual off) and compare
  forward speed / survival to the offline numbers in §1. Same solver ⇒ should match.
- **Default stays off** — zero risk to existing demos.

Reference: [`quad_mpc_offline.py`](../../projects/policies/research/mpc/quad_mpc_offline.py)
is the executable spec; the engine port just moves its rollout into the live solver.
