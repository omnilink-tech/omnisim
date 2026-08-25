# go2_turn — pod training plan (input for the RunPod campaign script)

Status 2026-07-17: the ghost is built, builder-gated 4/4 and `ghost_validator` **PASS**
(stamped into `projects/policies/ghosts/go2/go2_turn_ghost_lut.json`). Nothing below has
run yet. This file is the exact recipe the pod campaign should execute — it exists so the
campaign script does not have to reverse-engineer the manifest.

Read `cloud/runpod/README.md` (private ops tree — not in the public snapshot) first (watchdog, write-to-volume, TERMINATE, `GET /v1/pods`
must end `[]`). Both known pod traps apply here verbatim: GPU wheels into **both**
interpreters, and `--no-same-owner` on tar.

## 1. PRIMARY run — zero-hook (every knob exists in the recipe today)

```bash
cd $OMNISIM_HOME
bash projects/policies/training/run_quad_walk_rl.sh 0 go2_turn_r1 train headless \
  QUAD_ROBOT=go2 \
  QUAD_GHOST=projects/policies/ghosts/go2/go2_turn_ghost_lut.json \
  QUAD_GHOST_FF=1 \
  QUAD_W_GMATCH=2.0 \
  QUAD_RES_SCALE=0.15 \
  QUAD_VX_TARGET=0.0 \
  QUAD_YAW=0.0 \
  QUAD_ENVS=16384 QUAD_FAST_RESET=1 MPC_NJMAX=64 MPC_NCONMAX=64 \
  QUAD_ITERS=400 \
  QUAD_SEED=0 \
  QUAD_WARMSTART=projects/policies/research/inference/policies/gpu_go2_shadow_r2_main/policy.pt \
  RES_POLICY=projects/policies/research/inference/policies/gpu_go2_turn_main/policy.pt
```

Why each non-default knob:

| knob | value | why |
|---|---|---|
| `QUAD_GHOST` | turn lut | corridor centre = the certified turn reference. The trainer asserts `robot: go2` + the exact joint order. |
| `QUAD_GHOST_FF=1` | | parity with the walk skill's decode (both BATON legs must decode identically). The turn lut's own corridor floor is 0.111 rad < the 0.15 corridor, so FF is margin here, not a necessity. |
| `QUAD_VX_TARGET=0.0` | | `r_vel` then pulls vx→0: turn **in place**. (`gp.vx` also →0, harmless in ghost mode — the analytic stride is not the centre.) |
| `QUAD_YAW=0.0` | **required** | the default `-0.5` prices `\|wz − wz_cmd\|` with `wz_cmd = 0` (no `QUAD_WZ_RANGE` set) — i.e. it would actively punish the very yaw rate the ghost commands. Zero it. The turn is carried by corridor centre + gmatch; obs slot 48 stays 0.0, which is **exactly what today's deploy host feeds** (`go2_baton_deploy.py` hardcodes `[0.0]`), so training and deploy stay obs-consistent with **no code changes anywhere**. |
| `QUAD_W_GMATCH=2.0`, `QUAD_RES_SCALE=0.15`, envs/fast-reset/njmax | | verbatim from the shipped `go2_shadow_walk` recipe. |
| `QUAD_WARMSTART` | r2 shadow champion | best Go2 policy in the repo (0.415 m/s, 99.7 % never-fell), same 48-dim obs / 12-dim action contract. Disclosure: this makes the result an *upgrade* experiment, not from-scratch. |
| `RES_POLICY` | `.../gpu_go2_turn_main/policy.pt` | the recipe saves there and auto-exports `policy.onnx` beside it (`_export_onnx`) — which is the exact `policy.checkpoint` path the manifest and the sequence draft already reference. |

Cost shape: 400 iters × rollout 12 × 16 384 envs ≈ 79 M env-steps ≈ minutes on a 4090 at
the measured ~780 k steps/s. Cheap enough to run a seed pair (`QUAD_SEED=0/1`) if the
first result is ambiguous.

## 2. Eval criteria (in-engine eval prints at the end of training)

The recipe's eval reports `vx / never_fell / fwd / |ydrift| / gmatch`. For a turn:

- **never_fell ≥ 99 %** (the walk lineage hit 99.7 %; a statically stable turn should not be worse),
- **gmatch ≥ 0.85** (the lut's PD self-match ceiling is 1.000, so headroom is real),
- **vx ≈ 0, fwd ≈ 0, |ydrift| small** — a turn-in-place that translates is failing its own spec.

⚠️ the in-engine eval does **not** report yaw rate. The binding turn number comes from the
deploy exam (below) — do not claim a turn rate from the eval line.

## 3. Deploy exam (after the champion lands)

Solo-turn schedule on the baton world, headless, Newton sidecar checked:

```bash
BATON_SCHEDULE="walk:6,turn:20,walk:6" bash scripts/dev/run_go2_baton_deploy.sh 40
```

with the sequence draft's env (`skill_lib.py sequence go2_walk_turn_walk --dry-run` prints
the full exported bundle). PASS bar: `ONNX loaded:` on BOTH policies (never trust the exit
code — the zero-residual bare-ghost trap), sustained yaw ≥ 0.5 rad/s during the turn leg,
|xy drift| < 0.3 m over the leg, 0 falls, and **read roll/bz next to gmatch** (a flipped
Go2 has scored gmatch 0.92 on its back).

**Blocked on one deploy hook even for the zero-hook plan**: `go2_baton_deploy.py` line
`gait_t += dt * (1.0 if st.target == "walk" else 0.0)` freezes the gait clock in mode
`turn`, which would hold one lut bin and degenerate the turn into a twisted stance. Needed:
advance the clock for `turn` too (a hold stops the clock; a cyclic leg never does), e.g.
`st.target in ("walk", "turn")`. That file is currently being edited by another workstream
— coordinate, do not race it.

## 4. UPGRADE plan — only if the primary champion under-rotates

Symptom: deploy yaw < 0.6 × the ghost's `wz_meas` (0.5895 rad/s) with good gmatch — i.e.
the policy tracks the pose but leaks the rotation through stance slip.

Recipe hook (in `quad_walk_recipe.py`'s `GhostQuadWalkEnv`, ~6 lines — **owned by another
workstream**, spec only):

- `__init__`: `self._wz_fixed = float(os.environ["QUAD_WZ_FIXED"]) if os.environ.get("QUAD_WZ_FIXED") else None`
- `_reset_envs`, after the existing RSI block, guarded like `_ghost_leg` for the
  construction-time call:
  ```python
  if getattr(self, "_wz_fixed", None) is not None:
      idx = (torch.arange(self.n, device=self.tdev) if env_mask is None
             else torch.nonzero(env_mask, as_tuple=False).squeeze(-1))
      self.wz_cmd_t[idx] = self._wz_fixed
  ```

Effect: obs slot 48 carries the command and `r_yaw = QUAD_YAW·|wz − wz_cmd|` becomes a
true yaw-tracking reward. Then train with `QUAD_WZ_FIXED=0.5895 QUAD_YAW=-1.0` (rest as in
§1) — and the deploy needs the second hook too (wz obs slot = morph-blended `wz_meas` of
the active legs instead of the hardcoded 0.0), or the policy runs out-of-distribution at
deploy exactly the way the zero-residual trap taught us to fear.

## 5. Register / ship

1. `skill_lib.py freeze go2_turn --from projects/policies/training/runs/go2_turn_train.env.json`
   (or hand-edit `train.frozen_env`) once the run reproduces.
2. Copy `launch_env.txt` conventions from `gpu_go2_shadow_r2_main/` (tag + eval line).
3. Flip `status` open→experimental only after the deploy exam, and →verified only after
   the full `go2_walk_turn_walk` sequence runs live with 2 clean switches.
