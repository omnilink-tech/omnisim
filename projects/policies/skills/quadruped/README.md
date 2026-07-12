# Quadruped skills

Quadruped skills follow the same skill contract as the humanoid ones
([../README.md](../README.md)) — named, composable, with a manifest + verification
record — but they live in a **different implementation regime**, deliberately:

| skill | regime | status | basis |
|---|---|---|---|
| `stand` / posture / `balance` | deterministic | available (per-robot stand controllers) | stiff stand + capture-point/CoP |
| `walk` / `trot` / `turn` | **RL** | shipped for Spot / Go2 / B2; re-hosted Unitree | residual PPO on a model-based gait |
| `get_up` / recover | RL (Shadowing) | Spot/B2 rise solved; durable hold fragile | ghost-tracking pipeline |

Why RL for locomotion: a measured deterministic quad-walk attempt showed the RL
residual does nearly all the propulsion (bare trot model drifts; a deterministic
Raibert controller roll-flips). Owner direction is to **keep RL for quadruped
locomotion** and let deterministic control own stand/balance/posture. See
`docs/developer/rl-current-state.md` and the empirical re-verification notes.

So the quad skill library is a *mix*: deterministic stand/posture/recover-hold
manifests + RL walk/trot/turn manifests wrapping the proven policies
(`projects/policies/research/...`). The skill *interface* is shared with the
humanoid library; the implementations are not.

**Shipped skills:** `go2_walk` and `spot_walk` (RL residual PPO on a model-based trot;
`skill.json` in the sibling dirs). They deploy via `deploy.run: "world"` — the runner
launches the baked-in-controller `.wbt` rather than the g1 recipe launcher — so
`python ../skill_lib.py run go2_walk` works cross-robot. Their ghosts use the
quadruped **generated / .npz lineage** (a different serialization than the humanoid
json luts; `ghost.format` flags it, and `ghost_lut.py` does not validate them). More
quad skills (turn, get-up, terrain) follow the same pattern.
