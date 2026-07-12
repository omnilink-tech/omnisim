# Shadowing — claim-by-claim verification (2026-06-23)

> **What this is.** An independent, *regenerable* verification pass over every claim in
> [shadowing.md](shadowing.md), run on this checkout (binary built 2026-06-23 18:06,
> Newton/mjwarp + CUDA available). Where the paper and the canonical
> [rl-current-state.md](rl-current-state.md) disagree, **rl-current-state.md wins** and the
> paper headline is flagged. Every PASS below cites a command you can re-run; no number
> here is "documented-only" unless explicitly tagged.
>
> **Headline outcome:** the paper's three method contributions (generator → verifier →
> tracker) and its central *mechanism* (the feasibility certificate separates shadowable
> from un-shadowable ghosts) **reproduce cleanly**. Verifying them surfaced a **real bug**:
> the certificate's *main entry point* (`ghost_verifier.verify`) **false-rejected every
> feasible locomotion ghost** — it could not tell a feasible walk from a levitating
> impossibility. Fixed here (see §3). The paper's deploy *distances* are a mixed bag:
> **Go2 walk verified to +111 m / 0 falls**, but the **"G1 walk 200 m+, zero falls"
> headline does not reproduce** (finite bout — consistent with rl-current-state.md, not
> with shadowing.md §Contributions-4).

## 1. Method (regenerate everything)

```bash
# Component 1 — generator produces a feasible-by-construction ghost (fresh MPPI roll)
python projects/policies/research/shadowing/generate_go2_walk.py

# Component 2/3 — the feasibility certificate, across morphologies, as a SELF-CHECKING test
python projects/policies/research/shadowing/verify_certificate_suite.py        # exits non-zero on any mismatch

# Component 2/3 — the dynamic certificate's own A/B (feasible walk vs levitating)
python projects/policies/research/shadowing/ghost_verifier_dyn.py

# Main entry point (now routes legged ghosts to the dynamic certificate)
python projects/policies/research/shadowing/ghost_verifier.py --npz _scratch/go2_walk_ghost_generated.npz \
    --mjcf projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml
```

## 2. Claim-by-claim matrix

| # | Claim (shadowing.md) | Verdict | Evidence (this checkout) |
|---|---|---|---|
| **C1** | Generator = receding-horizon MPPI over MuJoCo, control = position-target actuators → *feasible by construction* | **VERIFIED** | Fresh `generate_go2_walk.py`: ghost walks +0.81 m, **no fall over 3 s**, peak torque 100 % of limit (within limits, saturated 13/150). |
| **C2** | Robot-*agnostic* generator: interface is strictly `(model, intent) → ghost` | **VERIFIED (structurally)** | One `GhostGenerator` class drives go2 / b2 / spot / g1 / arm generators; only the MJCF + `Intent` differ. Go2 re-rolled fresh; b2/spot/g1/arm ship stored ghosts. |
| **C3** | A numerical, **RL-independent feasibility certificate**, run *before* training | **VERIFIED + FIXED + IMPROVED** | The contact-wrench LP (`verify_legged`) cleanly separates feasible (base residual ≈0 % mg) from infeasible (levitate = 101 % mg) on go2/b2/g1, and the arm cert separates in-reach from beyond-reach toss. **But the main `verify()` entry was broken (§3).** |
| **E1** | Feasible vs infeasible ghost → RL success vs failure (same robot/learner) | **VERIFIED at the certificate level** | `verify_certificate_suite.py`: feasible walk PASS / lifted-twin FAIL for go2, b2, g1 — a controlled A/B (identical joints, only base dynamic-consistency differs). Full RL-outcome A/B is consistent with the deploy evidence (E4) + the documented sit-stand parking. |
| **E2** | Feasibility *score* predicts learnability across many ghosts | **PARTIAL** | The scalar score is now meaningful and attributable (§3); a full score-vs-RL-success correlation study (many policies) was **not** re-run here. |
| **E3** | Generality incl. a **non-legged manipulator** (fixed-base arm toss beyond reach) | **VERIFIED at the certificate level** | The arm toss PASSes (score 0.08), the beyond-reach toss FAILs (score 0.00 — task-infeasible reach). The deploy landing-accuracy (1.5 cm) was not re-run. |
| **E4** | Sim-to-deploy: G1 walk 200 m+, Spot/Go2 walk, arm toss — *all deployed* | **MIXED** | Quadrupeds ✅: **Go2 +111.45 m @ t=293 s, 0 falls** and **Spot +170.8 m @ t=426 s, 0 falls**, both dead-straight, upright, Newton/mjwarp confirmed. **G1 walk ❌ does NOT reach 200 m** — see §4. Arm toss landing not re-run. |
| **E5** | Boundary: feasible ref + tracking is necessary-but-not-sufficient for an unstable contact-rich *initiation* (G1 dead-seated launch) | **NOT RE-RUN (consistent)** | The 21 PPO + DAgger runs are historical; matches rl-current-state.md. No new evidence either way. |
| **Q1** | "**G1 walk 200 m+, zero falls**" (Contribution 4 headline) | **CONTRADICTED** | rl-current-state.md: finite ~34 s bout (incorrect-COM default) / ~12 s (corrected COM); 297/212/340 m are trainer/old-path numbers. My headless run §4 confirms the finite bout. |
| **Q2** | Stand-and-wave: **987/1000 survival in the trainer** | **NOT RE-RUN** | Trainer-side claim; would need a GPU training run. |
| **Q3** | G1 sit-stand body tracks **to 2.5 cm** | **documented-only** | Not regenerable from a committed artifact (per rl-current-state.md). |
| **Q4** | Throughput 16384 envs, ~500 k steps/s, 1.57 B in ~53 min (H100/H200) | **NOT RE-RUN** | No H100 on this box. |

## 3. Bug found and fixed — the certificate's main entry point false-rejected feasible walks

**Symptom (reproducible before the fix).** On a *feasible, by-construction* Go2 walk ghost,
the main entry point returned **FAIL**, and returned **FAIL** on a physically-impossible
*levitating* ghost too — i.e. it could not distinguish them, the exact opposite of
Contribution 3:

```
# ghost_verifier.verify() BEFORE the fix, on the feasible go2 walk:
[open-loop PD] FALLS -> unsustainable
[inverse-dyn] base residual: max=370.5N (=235% of mg) ... actuated torque: 355% of limit -> INFEASIBLE
[verify] FAIL -- collapses open-loop
```

**Root cause.** `ghost_verifier.verify()`'s floating-base branch gated on (a) open-loop PD
sustainability — which *every* dynamic gait fails (a walk is open-loop unstable; it balances
via state-dependent foot placement) — and (b) an inverse-dynamics cert its own docstring
marked **EXPERIMENTAL** ("over-reports … needs contact handling"). The *correct* certificate,
the per-step contact-wrench LP `verify_legged`, already existed in `ghost_verifier_dyn.py`
and its docstring literally said *"Wire it into `ghost_verifier.verify()`'s floating-base
branch."* That wiring had never been done; `generate_go2_walk.py` even carried a hand-rolled
"construction proof" + a NOTE apologising for the stock verifier's wrong answer.

**Fix** ([`ghost_verifier.py`](../../projects/policies/research/shadowing/ghost_verifier.py)):
route floating-base *legged* ghosts (detected via foot/ankle geoms) through `verify_legged`
as the **primary** certificate; keep the open-loop-PD result as a labelled secondary
diagnostic; keep the experimental ID cert only as the no-feet fallback. Added
`--contacts {auto,foot,real}` (`real` counts belly/knee ground contacts for get-ups), `--mu`,
`--contact-z`. After the fix:

```
# feasible go2 walk -> PASS (base 0.0% mg) ; levitating go2 -> FAIL (base 101% mg)
```

**Score calibration** ([`ghost_verifier_dyn.py`](../../projects/policies/research/shadowing/ghost_verifier_dyn.py),
[`ghost_verifier.py`](../../projects/policies/research/shadowing/ghost_verifier.py)): the old scalar
feasibility score collapsed to 0.00 the instant any single margin was exhausted, so it read
0.00 even for a comfortably-*passing* walk. Recalibrated to per-constraint satisfaction
relative to each PASS threshold, combined by the binding constraint and **reported by
component** (`s_base`, `s_torque`, `s_contact`). The arm score now also folds in the
task-reach margin, so PASS/FAIL and the score no longer disagree (beyond-reach toss → 0.00).

**Regression guard.** New self-checking suite
[`verify_certificate_suite.py`](../../projects/policies/research/shadowing/verify_certificate_suite.py)
asserts feasible-PASS / infeasible-FAIL across go2/b2/g1 + the arm A/B and **exits non-zero
on any mismatch** — turning the central empirical claim into a runnable test instead of prose.

Certificate suite result (this checkout):

```
go2 walk (feasible)        PASS 0.69 | levitating FAIL 0.00
b2  walk (feasible)        PASS 0.50 | levitating FAIL 0.00
g1  walk (feasible)        PASS 0.05 | levitating FAIL 0.00   (g1 is torque-tight by design)
arm toss (in reach)        PASS 0.08 | beyond reach FAIL 0.00
-> ALL CORRECT
```

## 4. The G1 "200 m+" claim — honest result

**It does not reproduce on this checkout — in two independent ways:**

1. **The policy that backs the *finite* bout is absent.** rl-current-state.md's reproducing
   G1 walk is `gpu_newton_g1_walk_ft_pdoff_clamp` (+5.9 m / `FALL@33.82 s`). That ONNX is
   **not in this clone** (`g1_deploy_launch.py` resolves the path, but the file is missing);
   only the older `gpu_g1_walk*` mjwarp lineage is present (203 G1 ONNX files). So the 34 s
   bout — let alone 200 m — cannot be regenerated here.

2. **The present, deployable G1 walk policy topples in ~1.3 s.** I ran the policy the standard
   launcher points at, `gpu_g1_walk15_c12` (legs-only, documented "+25.9 m / 68.5 s, zero
   falls"), headless under Newton/mjwarp with its exact matching env. Engine confirmed Newton
   (`[WbNewtonBackend] warp + newton imports OK`, 14 dynamic bodies). Result:
   **`FALL@1.31 s`** — pitch ≈ −1.4 rad (≈ 80° forward), `bz` collapsed 0.79 → 0.06,
   ~0 forward progress, then lies on the ground (cmd: `_scratch/g1_c12_deploy.log`).

**Verdict.** The paper's "G1 walk 200 m+, zero falls" (shadowing.md §Contributions-4 / E4) is
**not supported** here. The honest picture matches rl-current-state.md: the G1 deploy walk is
a fragile, finite bout, and the headline distances are trainer/old-path numbers. *Caveat:* the
c12 run was a **cold first load** (the [cold-first-load trap](rl-current-state.md) — articulation
under-tracks on load); the G1 walk controller does not `warmup_reload`, so a warm session may do
better — but the policy that reportedly reaches 34 s is not present to test, and 200 m is not on
the table regardless.

**Contrast — the quadruped claim holds, on the same engine/binary.** Go2 walk **+111.45 m**
and Spot walk **+170.8 m**, both 0 falls, dead-straight, upright, Newton/mjwarp confirmed
(`world finalised (solver=MuJoCo (mujoco_warp))`). These are extended single bouts; the
canonical [`rl-current-state.md`](rl-current-state.md) cites shorter-window headline figures
(Spot **+30 m** / Go2 **+66 m** / B2 **+95 m**, 0 falls). Where a single headline number is
needed, defer to `rl-current-state.md`. A standing biped is an inverted pendulum
that exposes every sim-to-deploy discrepancy; a statically-stable quadruped does not. The
pipeline crossed the deploy gap for quadrupeds (Go2 + Spot verified here, B2 per canonical
2026-06-23), **not** for the G1 walk.

## 5. Hardening — a motion-agnostic certificate (2026-06-23)

§3's certificate was clean only for the **headline class** (flat-ground foot-contact walks). It
**under-reported feasibility** on every other contact-rich motion — get-ups (belly/knee support,
no feet on the ground), ballistic jumps (a real flight phase tripped the airborne gate), hill
walks (feet at world `z=h(x)>0` read as off-ground), and chair-supported sit-stands. It also
**missed Spot entirely** (Spot has no `foot`-named body, so the name-matched foot set was empty).

A new additive module — [`projects/policies/research/shadowing/feasibility_certificate.py`](../../projects/policies/research/shadowing/feasibility_certificate.py),
`certify(ghost, mjcf, motion='auto') -> (passed, score, metrics)` — fixes this and is now the
**default** certificate the main `verify()` entry routes legged ghosts to (falls back to the core
LP / open-loop gate if unavailable). The committed `verify_legged` walk path is preserved
byte-for-byte. Four changes: (1) **size-aware bottom-surface support-contact reconstruction** over
*all* robot collision geoms against a terrain-aware ground (covers belly/knee/elbow support, Spot,
external chair surfaces); (2) **flight-aware base feasibility** — true free fall (`‖rhs[:6]‖≤flight_tol·mg`)
is feasible, levitation (≈mg against nothing) FAILs; (3) **terrain-aware ground** (`ground_h(x,y)`;
hills run dynamics in the injected terrain MJCF); (4) a **margin layer** — CoP/ZMP edge distance,
required friction, torque headroom, and **capturability** (reported, *not* gated — the documented
necessary-not-sufficient signal). It also returns **INDETERMINATE** when support can't be
reconstructed — never a vacuous PASS.

### Verified across the repertoire (`python projects/policies/research/shadowing/feasibility_certificate.py` → ALL CORRECT, exit 0)

| class | cases | verdict |
|---|---|---|
| walk | go2 / b2 / g1 | **PASS** (byte-match to the committed LP: 0.67 / 0.47 / 0.00) |
| get-up | spot / b2 | **PASS** (judged on reconstructed leg support) |
| crouch / sit | spot crouch / g1 sit-stand (chair) | **PASS** |
| hill | b2 hill6 / spot hill8 | **PASS** (against the injected terrain model) |
| jump | spot pronk | **FAIL → honest boundary** (see below) |
| arm | arm toss / toss-far | **PASS / FAIL** |
| negative controls | go2 levitate / frozen-apex jump / arm toss-far | **FAIL** |
| **adversarial** | real getup joints + 8 Hz base shake / 3 m/s drift | **INDET / FAIL** (never PASS) |

### Bug found in adversarial review — and fixed

The first hardened draft **rubber-stamped get-ups**: its partial-stance transition mask dropped
*every* judged frame (b2 getup: `n_stance=0`), so it PASSed on an empty stance array — and grafting
a physically-impossible base trajectory (8 Hz shake, 3 m/s drift) onto real getup joints **still
PASSed**. Fixed two ways, both now regression-guarded in the self-test: (a) exclude only a tight
±2 window around contact-count *switches* (not all partial-stance frames — a get-up is *entirely*
partial stance) plus the takeoff/landing impulse adjacent to a flight phase; (b) an **abstain
guard** — if too few support frames are actually judged (e.g. b2 has no trunk collider), return
**INDETERMINATE**, never PASS; the guard never softens a clear infeasibility (a levitation signature
still FAILs).

### Honest remaining caveats (do not over-read)

- **The scalar score is a soft margin, not a learnability regressor.** `s_tau` pins to ~0 when any
  one DOF rides its torque limit, so genuinely-feasible torque-tight ghosts (g1 walk, hill walks)
  score ≈ 0 yet correctly PASS. **Trust the binary verdict; treat the scalar as a margin.**
- **Margins are read from a non-unique LP primal** (the objective constrains only the base slack),
  so `tau_frac`/`req_mu`/`fric_slack` are directional indicators, not bounds; CoP/ZMP and capture are
  correct/monotone.
- **The jump is an honest boundary, not a tuned PASS.** The explosive pronk's *kinematic* ghost is
  not contact-consistent at the push-off (penetrating feet + finite-diff base accel → 33% mg
  unexplainable loading frames). The certificate **FLAGS** it rather than rubber-stamp; a clean
  certification needs the generator's *recorded* contact forces, not a kinematic snapshot.
- **b2 get-up rests on a model gap:** `b2_planner.mjcf.xml` has no trunk/belly collider, so the
  belly-supported phase is certified on leg contacts + robust aggregation; adding a trunk collider
  would let it pass on a fully-reconstructed support polygon.
- Degenerate inputs (NaN/`dt=0`) raise rather than FAIL gracefully — the failure mode is
  under-reporting, never admitting an impossible motion as PASS.

## 6. E2 — does feasibility predict deploy success? (`e2_feasibility_vs_outcome.py`)

The paper's headline E2 claim, run for the first time. **Result (honest):** the certificate is a
**pre-training triage, not a deploy oracle** — its predictive content is a one-way partition of
failures by *root cause*:

```
cert FAIL                -> the ghost is infeasible            -> FIX THE REFERENCE
cert PASS + deploy FAIL  -> ghost fine; trainer<->deploy gap   -> FIX THE TRACKER / SIM2REAL
```

The only cert-PASS-but-deploy-FAIL *legged* case in hand is **G1 walk** — feasible ghost, yet the
policy topples (§4) — i.e. a sim2real/durability gap, **not** a reference defect. The impossible
controls are caught *before* any compute is spent. **Honest limits:** n is tiny (11 ghosts, 8 with a
known deploy verdict); the deploy outcomes are documented-only (per [rl-current-state.md](rl-current-state.md));
no cert-FAIL-but-tracked and no independently-confirmed cert-PASS-but-unlearnable case exists yet.
A full study needs ≥20–30 ghosts spanning a graded feasibility range, each with a trained policy + a
logged deploy-survival metric. `e2_feasibility_vs_outcome.py` is the harness; today it reports the
partition, not a feasibility→learnability curve.
