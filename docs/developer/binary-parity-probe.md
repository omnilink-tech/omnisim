# Binary-level train↔deploy physics parity probe

**Status (2026-06-26): built, run, and the chaos-free lane PASSES at machine
precision.** For the first time, the *real* `omnisim-bin` binary's per-tick
trajectory is compared against the trainer's — not the `g1_deploy_runtime.py`
Python extract. In the **welded-base lane** (after fixing the `staticBase` bug
below) the binary and the trainer step a deterministic, no-RL G1 sweep
**identically to ~1e-5 rad (median 4.7e-5 rad / 0.003°, settling to the float32
floor), base error exactly 0** — i.e. the same physics to machine precision. (The
free-base lane is looser, ~0.15° median, purely because an unstable inverted
pendulum amplifies float32 round-off — not a model gap; the welded lane removes
that chaos.) The deploy's compiled model is also **dynamically equivalent** to the
trainer's (same total + per-segment mass; fixed-link fusion is exact). This
**certifies the physics/pipeline-parity gap at the binary level**; it does **not**
close the *durability* gap (see [train-deploy-gap.md](train-deploy-gap.md) — that
is a separate problem and needs the Unitree recipe, not physics mending).

---

## Per-robot status (welded lane, binary vs trainer)

The probe is now **robot-agnostic** ([`parity_probe_spec.py`](../../projects/policies/research/backends/parity_probe_spec.py)
+ [`robot_parity_probe.py`](../../projects/policies/research/training/robot_parity_probe.py) +
the `parity_probe` controller), driven by each robot's deterministic-stand spec.
Run any robot with `scripts/dev/run_parity_probe.ps1 -Robot <name> -Compare`.

| Robot | welded-lane leg parity (probe-phase median) | verdict | note |
|---|---|---|---|
| **G1** (23-DOF humanoid) | ~5e-3 → settles ~1e-5 rad | ✅ PASS | machine precision |
| **H1** (19-DOF humanoid) | **6e-8 rad** (→1e-9 by end) | ✅ PASS | machine precision |
| **Valkyrie** (32-DOF humanoid) | legs **1e-5 rad** | ✅ legs PASS | neck/torso **ring** at the over-stiff uniform ke=2000 on near-massless distal joints — *oscillatory numerical conditioning, not a model gap* (legs prove the model matches) |
| **Go2** (12-DOF quadruped) | **4e-4 rad** (0.024°) | ✅ PASS | residual is the deploy's underdamped gains (kd=6) amplifying float32, not a model gap |
| **B2** (12-DOF quadruped) | **2e-4 rad** | ✅ PASS | machine-near |
| **Spot** (12-DOF quadruped) | **4.5e-7 rad** (→1e-8 by end) | ✅ PASS | machine precision **after the self-collision fix** (was hip_x ~3.4° DIFF — see below) |

Takeaway: the engine + compiled model are parity-clean **in the binary** for all 6
legged robots probed — machine precision for G1/H1/Spot, ~0.0001–0.02° for Go2/B2,
and legs-machine-precision for Valkyrie (its only divergence is the near-massless
**neck/torso ringing** at the over-stiff uniform ke=2000 — a gain/timestep
conditioning artifact, not a model gap). No wholesale engine gap on any robot.

### The Spot hip_x gap — root-caused + fixed (self-collision)

Spot first showed a localized hip_x (ab/adduction) divergence of ~0.06 rad (3.4°)
while hip_y/knee matched. Root cause (found by diffing the two compiled `MjModel`s —
they were **byte-identical** on every static field: mass, COM, inertia, anchor,
axis, actuator gains): the **trainer had 4 self-collision contacts at rest, the
deploy had 0**. newton's `add_urdf` defaults to `enable_self_collisions=True`, so
the trainer's full-mesh Spot legs touched the body at the crouch and got shoved
(hip_x most); the deploy (`WbNewtonBackend`) **filters intra-robot self-collision**,
so it had none. Fix: pass `enable_self_collisions=False` in
[`robot_parity_probe.py`](../../projects/policies/research/training/robot_parity_probe.py) to match
the deploy. Spot then matches to ~4.5e-7 rad (PASS); H1/Go2/B2 are unchanged (they
didn't self-collide at their poses). This is a probe-harness fidelity fix, not an
engine bug — but it's the kind of trainer↔deploy discrepancy the probe exists to catch.

---

## Closed-loop POLICY validation (train == deploy end-to-end)

The open-loop probe proves the *physics* steps identically. The closed-loop check
proves a *trained policy* behaves identically: same ONNX + same obs + same action
law + (certified) same physics => same trajectory. Harness: the trainer eval
dumps env-0's closed-loop trace (`gpu_mjwarp_g1_stand_trainer.py --eval
--dump-trace`), a certified-path eval runs the same ONNX through `add_urdf` +
`SolverMuJoCo` ([`g1_policy_eval_addurdf.py`](../../projects/policies/research/training/g1_policy_eval_addurdf.py)),
and the `g1_policy_probe` controller runs it in the binary; `g1_parity_compare
--policy` diffs them. Obs is **position-only** (`q-NOMINAL`, proj_gravity,
last_action — 29-dim): dropping joint velocity + base linear velocity removes the
two hardest-to-reproduce obs terms.

A minimal G1 stand was trained (29-dim, gains forced to the deploy's 400/60,
clamp-exported) in ~4 min on the local GPU.

**Result (welded base, chaos-free): PASS.** The same ONNX run closed-loop in the
certified `add_urdf` path and in the **real binary** matches to **2.7e-4 rad
(0.016°)** over 200 ticks (3.2 s), never exceeding 1e-3; the policy's commanded
targets match to **5e-5 rad**. The obs→network→action→physics loop is identical in
train and deploy.

**Two findings the closed-loop check surfaced:**
1. **Free-base is Lyapunov-chaos-limited, not a parity bug.** The trained stand is
   marginal (falls ~1 s); once it tips, *any* float-level difference amplifies
   exponentially, so a free-base falling policy diverges between train and deploy
   regardless of how well the physics matches. The welded lane (same harness)
   matching to 0.016° proves the divergence is chaos, not a pipeline error. A clean
   free-base closed-loop proof needs a **durably non-falling** policy.
2. **The team's training physics path doesn't match the binary; the certified one
   does.** The GPU stand trainer steps **raw `mujoco_warp` on an MJCF**
   (`mjw.step`); the binary steps **`newton SolverMuJoCo`** (`add_link`). In the
   pre-tip window the certified `add_urdf`+`SolverMuJoCo` path tracks the binary to
   ~0.02 rad while the raw-mjw-MJCF trainer is 0.08→0.34 rad (tick-0: **0.7° vs
   4.5°**). This is the repo's known `mjw.step() != SolverMuJoCo.step()` lesson,
   now measured end-to-end: **train via `SolverMuJoCo` (add_urdf), not raw-mjw-MJCF.**

---

## Why this exists (the hole it closes)

Every prior G1 trainer↔deploy parity proof
([`g1_golden_parity.py`](../../projects/policies/research/training/g1_golden_parity.py),
[`test_g1_physics_spec_conformance.py`](../../tests/test_g1_physics_spec_conformance.py))
compares a Python-built trainer model against
[`g1_deploy_runtime.py`](../../projects/policies/research/backends/g1_deploy_runtime.py) — a
Python *extract* of the C++ `kNewtonRuntimeSource` — and steps **both through the
same in-process solver**. The extract is kept faithful only by a string-equality
test ([`test_g1_deploy_runtime_sync.py`](../../tests/test_g1_deploy_runtime_sync.py)).
**Nothing ever put a `.wbt` through the real binary and compared the trajectory.**
The Tier-2 golden-trajectory test in the conformance suite is an explicit
unimplemented TODO ("requires the native OmniSim binary… cannot run on a CPU CI
runner"). This probe is that test, run as a local/GPU lane.

The probe is a **deterministic, no-RL controlled experiment**: the identical
scripted joint-target sequence is applied in the trainer Python solver and in the
deploy binary, and the per-tick trajectories are diffed. Because there is no
policy and no stochasticity, any divergence is pure model/stepping physics.

## Components (all single-sourced through the spec)

| Piece | File |
|---|---|
| Probe spec (pose, gains, settle, sequence) + `probe_targets()` / `probe_settle_target()` | [`g1_physics_spec.py`](../../projects/policies/research/backends/g1_physics_spec.py) (`probe` block in [`g1_physics.json`](../../projects/policies/research/backends/g1_physics.json)) |
| Trainer harness (`add_urdf` build + EXACT deploy solver) | [`g1_parity_probe.py`](../../projects/policies/research/training/g1_parity_probe.py) |
| Deploy controller (runs inside omnisim-bin; trace via supervisor — no C++ rebuild) | [`research/controllers/g1_parity_probe/`](../../projects/policies/research/controllers/g1_parity_probe/g1_parity_probe.py) |
| Deploy worlds (welded + free base) | [`g1_parity_probe.wbt`](../../projects/policies/research/worlds/g1_parity_probe.wbt) / `g1_parity_probe_free.wbt` |
| Numerical diff (+ CI-able stable-window verdict) | [`g1_parity_compare.py`](../../projects/policies/research/training/g1_parity_compare.py) |
| Launcher | [`run_g1_parity_probe.ps1`](../../scripts/dev/run_g1_parity_probe.ps1) |
| CI guards (pure-python parts) | [`tests/test_g1_parity_probe.py`](../../tests/test_g1_parity_probe.py) |

The probe pose (deep squat) + the stand gains (KE 400 / KD 60) used to live as
hand-synced literals (the deploy stand used deep-squat/KE400 while the trainer
probe `build_g1_native` used shallow-squat/KE20 — exactly the class of
trainer↔deploy mismatch this probe surfaces). They are now one source.

## Run it

```powershell
# Welded-base sweep, both sides + diff (the chaos-free machine-precision lane):
powershell -File scripts/dev/run_g1_parity_probe.ps1 -Sequence sinusoid -Settle 40 -Ticks 80 -RecordSettle -Compare
# Free-base deterministic stand (realistic; ~0.15deg, chaos-limited):
powershell -File scripts/dev/run_g1_parity_probe.ps1 -Free -Sequence hold -Settle 40 -Ticks 80 -RecordSettle -Compare

# Diagnostics: -RecordSettle dumps the settle transient + the deploy's per-joint
# effective ke (OMNISIM_DEBUG_JOINTS) + the compiled model (OMNISIM_NEWTON_SAVE_MJCF).
# -NoLinkCom runs without the link-COM mend (A/B). -Gui to watch.
```

Both sides spawn straight-legged and ramp to the squat over an unrecorded settle,
converging to the same position-PD equilibrium — this removes the launch-IC
asymmetry (the trainer seeds the pose; the deploy spawns straight) so the recorded
window is a clean physics comparison.

## Result (2026-06-26)

**Welded-base lane (chaos-free, after the `staticBase` fix) — the headline:**
sinusoid sweep, 120 ticks, KE 400 / KD 60, `OMNISIM_NEWTON_USE_LINK_COM=1`:
**stable-window max 5.3e-3 rad, median 4.7e-5 rad (0.003°), settling to ~1e-5 rad
(float32 floor); base error exactly 0 → VERDICT PASS.** Only the single tick-0
spawn step shows ~1.9e-2 rad. This is the real binary and the trainer Python
stepping the same physics to machine precision.

**Free-base deterministic stand** (KE 400 / KD 60, `OMNISIM_NEWTON_USE_LINK_COM=1`),
120 ticks (~1.9 s), trainer `add_urdf` vs deploy `add_link`-from-`.wbt`:

- **joint angle: median ~2.5e-3 rad (0.15°), plateau ~5e-3 rad (0.3°)**, growing
  to ~1.7e-2 rad (1°) as the marginal inverted-pendulum stand amplifies float32
  round-off late in the window. Neither side fell.
- **base position: a few mm rising to ~2 cm; base orientation ≤ ~1.5°.**
- **Compiled-model equivalence (Phase 5, via `OMNISIM_NEWTON_SAVE_MJCF`):** the
  deploy's compiled robot mass is **28.03 kg = the trainer's exactly**. The deploy
  *fuses* fixed-jointed links (e.g. torso 8.562 + two stubs 0.244+1.036 → one
  9.844 kg body) — dynamically exact (no DOF between fused links). Total and
  per-segment mass are conserved. So the binary's model is dynamically equivalent
  to the trainer's, confirming the extract-based golden test represents the real
  binary.

Interpretation: the per-tick drift is float32 + chaos on a *marginal* stand, not a
model gap. The golden-parity harness already shows a *byte-identical* model
diverges ~0.49 m over 100 ticks for a **passive** free biped; the PD-stabilised
stand here diverges only ~2 cm because the controller damps the chaos. ~0.15–0.3°
is essentially the chaos floor for this system.

## Findings / open items

1. **`staticBase` weld-at-origin defect — FOUND by this probe and FIXED.**
   Symptom: with `staticBase TRUE` the **hip joints free-swung to their stops**
   (target −0.30, `ke=400`, yet `joint_q≈−2.3`) while ankles/knees/waist tracked
   fine. Root cause (bisected by loading the deploy's own `OMNISIM_NEWTON_SAVE_MJCF`
   dump in plain MuJoCo): the `staticBase` robot-root weld
   (`add_joint_fixed(parent=-1, child=root)` in `WbNewtonBackend.cpp`'s embedded
   runtime) **omitted `parent_xform`**, so the pelvis was welded at the world
   **origin** instead of its `.wbt` spawn pose. A base spawned at z=1.2 was pinned
   to (0,0,0) → the legs spawned 0.7 m through the floor → 38 contacts → huge
   constraint forces slammed the joints *off the root* (the hips; distal joints
   were shielded). With the pelvis welded at the correct height, plain MuJoCo holds
   the hips with this exact model and contacts = 0. Fixed-base arms (UR/Panda-style)
   hid this because they spawn near the origin **and** their first joint is a yaw (no
   gravity load).
   **Fix:** pin the weld at `body_q[root]` (mirroring the static-collider weld),
   in `WbNewtonBackend.cpp` + the regenerated `g1_deploy_runtime.py` extract. After
   the fix the welded lane PASSES at ~1e-5 rad (above). This bug affected **any**
   staticBase robot spawned away from the origin, not just the probe.
2. **Contact ke/kd is not the residual here.** Matching the deploy's
   `contact_ke=2500/kd=100/mu=1.0` on the trainer (`default_shape_cfg`, on by
   default in the harness) leaves the trainer trace byte-identical — the rigid
   stand contact is insensitive to it in this regime.
3. **Foundation fixed:** the RED `test_g1_deploy_runtime_sync` (the extract had
   drifted from the C++ by the `a340014c` welded-static inertia fix) was
   regenerated green via `_gen_deploy_runtime.py`, so every extract-based proof
   rests on a faithful extract again.
4. **Closed-loop OBS bug — FOUND by the closed-loop sibling test (2026-06-27) and
   FIXED.** The open-loop probe certifies *physics*; it does not exercise the
   *observation pipeline* a closed-loop policy reads each tick. A sibling test,
   [`closed_loop_parity_compare.py`](../../projects/policies/research/training/closed_loop_parity_compare.py),
   runs the SAME stand policy through the certified `add_urdf`+SolverMuJoCo path
   AND the real binary while **logging the obs vector each side fed the policy**,
   and diffs per component. It exposed that the base **angular-velocity** obs term
   was a different quantity in deploy: trainer reads mujoco free-joint `qvel[3:6]`
   (body frame), deploy fed `R^T·getVelocity()[3:6]` from the Newton backend —
   different frame/scale (norms 2.13 vs 1.67, yaw sign-flipped), diverging ~2 rad/s
   at tick 1 while the pose still matched to 3e-3. The policy got an
   out-of-distribution signal → toppled ~1.8 s. **Fix:** replace the engine
   ang-vel with the **finite-difference of `proj_gravity`** (reproducible to ~3e-3
   on both sides) in trainer, eval, and deploy. After the fix the from-scratch G1
   **stand stands 32 s / 0 falls in the binary** (commit `1416d52c`). Lesson: any
   deploy obs term sourced from the Newton `getVelocity()` angular part is suspect;
   derive rates from a reproducible orientation finite-diff.

## The boundary (read this)

This probe **certifies the physics/pipeline-parity gap at the binary level** — the
engine, the compiled model, and a deterministic open-loop trajectory all match to
the chaos floor. It is a necessary prerequisite and a clean diagnostic that
**separates the gaps**. **Update (2026-06-27):** the closed-loop sibling test
(finding 4) then proved that for the G1 **stand**, the *next* gap after physics was
**not** durability — it was a **reproducible-obs bug** (the engine base ang-vel),
fixable directly, after which the stand is durable in the binary. So the rule is
sharper now: with physics this close, when a from-scratch policy still falls in
deploy, **first run the closed-loop BUG-vs-CHAOS diagnostic**
([closed-loop-chaos-diagnostic.md](closed-loop-chaos-diagnostic.md)) — it classifies
the divergence as a real `[BUG]` (incl. obs bugs hiding under chaos), intrinsic
`[CHAOS]`, or clean `[MATCH]`. Only if the obs reproduce and it *still* falls is the
remaining problem durability (the Unitree obs/action/reward recipe,
[train-deploy-gap.md](train-deploy-gap.md) §4). The walk has **not** yet been put
through this test.
