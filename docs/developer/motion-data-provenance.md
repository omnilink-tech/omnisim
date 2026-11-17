# Motion-data provenance and licence rulings

**Status: RULING MADE 2026-08-22. Binding on the public snapshot.**
**This is not legal advice.** It is an engineering record of licence text we fetched, quoted and
reasoned about, so that a reader can check our reasoning rather than take our word for it.

This file exists because the public snapshot **squashes git history**. Before it was written, the
only record that OmniSim's G1 reference motions had a third-party dataset in their ancestry lived
in `projects/policies/training/data/SOURCE.md` — a file that is itself excluded from the snapshot.
The record of a problem must not be the one artifact that never ships. Whatever else changes, keep
this document in the public tree.

---

## 1. What this is about

OmniSim's Shadowing method trains a policy to track a **ghost**: a phase-indexed lookup table of
joint angles stored as `projects/policies/ghosts/<robot>/ghost_*_lut.json`. Ghosts come from four
different places, and only one of them is a third-party dataset:

| lineage | how the numbers were produced | third-party data? |
|---|---|---|
| **synthesised** | `ghost_synth*.py`, `build_step_turn_ghost.py`, FK/IK authoring — we plan the contacts and solve for everything else | no |
| **recorded-from-Unitree-official** | Unitree's released `motion.pt` walking policy replayed inside OmniSim, and the robot's own joint sensors sampled | Unitree policy weights (§6) |
| **LAFAN1-derived** | Ubisoft LAFAN1 mocap → community retarget to the G1 → blended into our references | **yes — this document** |
| **second-order re-record** | a policy trained against one of the above, replayed in-engine, its achieved motion sampled | derived question (§4, Class 3) |

Only the third and fourth rows carry a licence question. Everything else is ours.

---

## 2. The licence chain, verified

### 2.1 The root: Ubisoft La Forge LAFAN1

- Repository: <https://github.com/ubisoft/ubisoft-laforge-animation-dataset>
- Licence file: `license.txt` (13,135 bytes), fetched 2026-08-22 via the GitHub contents API.

The repository README states, verbatim:

> This dataset can be used under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0
> International Public License (see license.txt).

`license.txt` is the unmodified **CC BY-NC-ND 4.0** text. No Ubisoft-specific clauses were added.
The three clauses that decide this case, quoted verbatim from that file:

**Section 1 — Definitions**

> **Adapted Material** means material subject to Copyright and Similar Rights that is derived from
> or based upon the Licensed Material and in which the Licensed Material is translated, altered,
> arranged, transformed, or otherwise modified in a manner requiring permission under the Copyright
> and Similar Rights held by the Licensor.

> **NonCommercial** means not primarily intended for or directed towards commercial advantage or
> monetary compensation.

**Section 2(a)(1) — License grant** (this is the entire grant)

> Subject to the terms and conditions of this Public License, the Licensor hereby grants You a
> worldwide, royalty-free, non-sublicensable, non-exclusive, irrevocable license to exercise the
> Licensed Rights in the Licensed Material to:
> reproduce and Share the Licensed Material, in whole or in part, for NonCommercial purposes only;
> and
> **produce and reproduce, but not Share, Adapted Material for NonCommercial purposes only.**

**Section 3(a)(1)(C) — the decisive sentence**

> For the avoidance of doubt, You do not have permission under this Public License to Share Adapted
> Material.

"Share" is defined broadly enough to cover publishing to GitHub:

> **Share** means to provide material to the public by any means or process that requires permission
> under the Licensed Rights, such as reproduction, public display, public performance, distribution,
> dissemination, communication, or importation, and to make material available to the public
> including in ways that members of the public may access the material from a place and at a time
> individually chosen by them.

### 2.2 The retargeting layer

LAFAN1 is human mocap. Getting it onto a Unitree G1 required a retarget, which we did not do
ourselves.

- **Stated upstream:** `unitreerobotics/LAFAN1_Retargeting_Dataset` on HuggingFace.
  Fetched 2026-08-22: both the dataset page and the public metadata API
  (`https://huggingface.co/api/datasets/unitreerobotics/LAFAN1_Retargeting_Dataset`) return
  **HTTP 401 Unauthorized** to an unauthenticated client. It is **gated**. We have therefore never
  seen its terms, and cannot quote them.
- **What we actually used:** `lvhaidong/LAFAN1_Retargeting_Dataset`
  (<https://huggingface.co/datasets/lvhaidong/LAFAN1_Retargeting_Dataset>). Fetched 2026-08-22:
  HTTP 200, metadata API reports `"gated": false, "private": false`, and its **only** tag is
  `task_categories:robotics` — there is **no `license:` field** in its card metadata.

The mirror's dataset card ends with this line, verbatim:

> [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) is licensed under Creative
> Commons Attribution-NonCommercial-NoDerivatives 4.0 International Public License (unlike the code,
> which is licensed under MIT).

### 2.3 What that combination means

Read it carefully, because the mirror is testifying against itself:

1. The mirror **acknowledges** the source is **NoDerivatives**.
2. A retarget of mocap onto a robot skeleton is, by any reading, material that "is derived from or
   based upon the Licensed Material and in which the Licensed Material is translated, altered,
   arranged, transformed, or otherwise modified" — i.e. **Adapted Material**.
3. CC BY-NC-ND grants the right to *produce* Adapted Material but expressly withholds the right to
   *Share* it.

So the mirror is distributing Adapted Material that its own stated licence does not permit it to
distribute. **A distributor cannot convey a right it does not hold.** Nothing downstream of that
mirror — including every OmniSim artifact in §4 — inherits a redistribution right, no matter how
many transformations later.

The gated/ungated asymmetry is corroborating evidence rather than proof: the party closest to the
data (Unitree) put a gate on it, and an ungated third-party copy appeared anyway. That is the shape
of a copy made without the right to make it, and it is why "we got it from a public mirror" is not a
defence.

### 2.4 Why attribution cannot fix this

The obvious instinct is "add a NOTICE entry and ship it". That does not work here, and it is worth
being explicit about why, because it is the single most likely mistaken fix:

- **ND is not an attribution condition, it is a missing grant.** Section 3(a) attribution applies
  *"If You Share the Licensed Material"* — the unmodified material, for NonCommercial purposes.
  There is no attribution formula anywhere in CC BY-NC-ND that buys back the Share-Adapted-Material
  right, because that right was never granted. Compare CC BY, where attribution genuinely is the
  whole condition.
- **NC is independently fatal for this repository.** OmniSim is Apache-2.0, whose Section 2 grants
  every recipient a *"perpetual, worldwide, non-exclusive, no-charge, royalty-free, irrevocable"*
  licence to reproduce, prepare derivative works of, and **sell** the work. Shipping NC-only content
  inside an Apache-2.0 tree would promise downstream users rights the upstream licensor never gave
  us. That is true even for a purely non-commercial *fork* of OmniSim, because the promise is made
  by the LICENSE file, not by any particular user's intent.

Either clause alone is dispositive. Both apply.

---

## 3. Is a retargeted joint-angle trajectory a derivative of the source mocap?

**Our position: yes, with high confidence, for a direct retarget or blend.** Reasoning:

- Retargeting is a coordinate change plus a constraint solve. It preserves the thing that makes the
  clip valuable and identifiable — the timing, the weight shifts, the phrasing of a specific
  recorded human performance. Frame *n* of the retarget is a function of frame *n* of the source and
  nothing else; the mapping is dense and order-preserving.
- CC's own definition is deliberately broad — "translated, altered, arranged, transformed, or
  otherwise modified" — and a skeleton retarget is most naturally read as a *translation* of the
  same performance into a different body.
- The mirror itself treats the retarget as governed by LAFAN1's licence rather than as a new
  independent work: it reproduces the ND term on the retargeted dataset's own card.

The honest caveat: CC's definition ends with the qualifier *"in a manner requiring permission under
the Copyright and Similar Rights held by the Licensor"*, which loops back to copyright law rather
than settling it. Whether a *joint-angle time series* — arguably closer to a measurement than to an
authored work — attracts copyright at all is a real question, and jurisdictions differ (the EU's
*sui generis* database right, expressly named in this licence, exists precisely because raw data
sometimes does not). We do not need to resolve it: the licence covers "Copyright **and Similar
Rights** including … Sui Generis Database Rights", so the safe reading is the one we have taken.

---

## 4. The ghost store by lineage class, and the ruling for each

Every G1 ghost LUT declares its own ancestry in its `source` field, and that field is what the table
below is derived from — not filenames. Counts verified against the tracked tree on 2026-08-22.

### Class 0 — synthesised or planned from scratch (no third-party data)

**Ruling: SHIPS. No restriction.** Ours outright, Apache-2.0 with the rest of the repository.

`ghost_synth*`-built walks and stair climbs, `build_step_turn_ghost.py` turns, the FK/IK-authored
carry and crawl poses, and the hand-designed v3 family:

`ghost_walk_synth`, `ghost_walk_fast_v1`, `ghost_walk_backward64`, `ghost_carry_backward64`,
`ghost_carry_v1`, `ghost_carry_fast_v1`, `ghost_carry_mid_v1`, `ghost_carry_turn{,_pro45,_pro58}`,
`ghost_crawl_v1`, `ghost_kneel_synth`, `ghost_pushup_synth`, `ghost_squat_synth`,
`ghost_stair3_2step`, `ghost_stair_climb{,_solo,_synth,3_synth}`, `ghost_walk_climb`,
`ghost_step_turn{,_torso}`, `ghost_turn_90{,_slow,_preview,_solved,_solved_preview,_topp}`,
`ghost_turn_180`, `ghost_turn_natural90_closed`, `ghost_turn_pro90`, `ghost_turn_pro115`,
`ghost_stand_v1`, `ghost_v3c`, `ghost_v3d`, `ghost_v3e`, `ghost_v3f`, `ghost_v3g`, `ghost_walkstop`,
plus every quadruped ghost under `ghosts/go2/` and `ghosts/omniquad/`.

### Class 1 — recorded from the Unitree official policy

**Ruling: SHIPS, but see §6 — this is a separate, still-open question about Unitree's terms, not a
LAFAN1 question.** These contain no LAFAN1 data at all.

`ghost_unitree{,_u115,_u130}`, `ghost_official_arms`, `ghost_official_metric`,
`ghost_official_full{,_v2,_v3}`, `ghost_walk_mid_v1`, `ghost_h1_lut` (h1),
and the second-order re-records whose declared parent is the official motion rather than HOP-1:
`ghost_v5`, `ghost_carried`.

> `ghost_official_full_v3_lut.json` is the **flagship** G1 walk reference — the one
> `skills/humanoid/g1_walk` and every BATON sequence ride on. It is Unitree lineage. **The whole
> LAFAN1 ruling leaves the flagship demo untouched.**

### Class 2 — LAFAN1-derived, first order (**13 + 9 files**)

**Ruling: MUST NOT SHIP. Confidence: HIGH.** LAFAN1 sample values are arithmetically present in the
stored arrays.

Already denied before this review (9 files + directories):

| file | its own `source` says |
|---|---|
| `ghost_lafan1_walk1_win_lut.json` | `LAFAN1 retarget adapter: walk1_subject1.csv (lvhaidong/LAFAN1_Retargeting_Dataset, …)` |
| `ghost_lafan1_dance1_win_lut.json` | `LAFAN1 retarget adapter: dance1_subject1.csv (…)` |
| `ghost_pivot_turn{,_slow,_slow5}_lut.json` | `LAFAN1 retarget adapter: walk1_subject1.csv (…)` |
| `ghost_hum100_lut.json` | `humanized blend: 100% lafan1-walk1 cycle (frames 143..191) + 0% v3c` |
| `ghost_hum70_lut.json` | `humanized blend: 70% lafan1-walk1 cycle (frames 143..191) + 30% v3c` |
| `ghost_hum40_lut.json` | `humanized blend: 40% lafan1-walk1 cycle (frames 143..191) + 60% v3c` |
| `ghost_humedge_lut.json` | `humanized blend: 14% lafan1-walk1 cycle (frames 143..191) + 85% v3c` |

**Newly denied 2026-08-22 — the transitive set that was leaking (13 files).** A blend of a denied
artifact is itself denied:

| files | lineage arithmetic |
|---|---|
| `ghost_hop1_lut.json`, `ghost_hop1v9`, `ghost_hop1v9s`, `ghost_hop1v11`, `ghost_hop1v11_f13`, `ghost_hop1v11_f169`, `ghost_hop1v11_f169a`, `ghost_hop1v11_f169a2`, `ghost_hop1v11_f169a3`, `ghost_hop1v11_f169m`, `ghost_hop1v11_f169n`, `ghost_hop1v16` (**12**) | each declares `HOP-1 center: unitree->hum40 at alpha=0.375 incl cadence (1.156 Hz)`. `hum40` is 40 % LAFAN1-walk1, so **0.375 × 0.40 ≈ 15 % of every stored sample is a LAFAN1 value**, carried through two weighted averages. |
| `ghost_medley1_lut.json` (**1**) | `corpus medley: stand_intro.json + walk1.json + walk3.json + dance1.json` — LAFAN1 clip names, concatenated by `build_corpus_medley.py` (already denied). A concatenation is even closer to reproduction than a blend. |

These were the actual leak: they published while every file that documented their ancestry did not.

### Class 3 — second-order re-recorded from a HOP-1 reference (**23 files**)

**Ruling: DENIED, PRECAUTIONARY. Confidence that they are legally derivative: LOW-TO-MODERATE, and
genuinely unsettled.** This class deserves its own argument rather than being swept in with Class 2,
because the mechanism is categorically different.

What is physically in these files: the **robot's own motion**, sampled from its joint sensors while
the physics engine stepped, produced by a neural policy that had a HOP-1 LUT in its reward term.
Their `source` fields say so — e.g. `RE-RECORDED achieved motion (rule 4): policy=wr_v13_it250.pt
survivor 108/108 ticks -- from HOP-1 center: …`.

**The case that they are NOT derivative works:**

- No LAFAN1 sample value is arithmetically present. The chain passes through a lossy, non-invertible
  physical process: policy inference → PD servo → contact solve → sensor readback. You cannot
  recover a LAFAN1 frame from these arrays by any transformation.
- HOP-1 is only ~15 % LAFAN1 to begin with; the rest is Unitree official plus our own synthetic v3c.
  So the influence is on the order of a *sixth* of an input to a training reward.
- What survives the simulation is mostly **functional and factual**: cadence, gait phase, stride
  schedule. A biped walking at 1.156 Hz is a constraint of the mechanics, not an act of authorship,
  and functional elements are the least protectable part of any work.
- The performer is different (a 34 kg, 23-DOF robot on a weight-bearing balance harness), the medium
  is different, and the motion is measurably not the reference — the whole documented finding of
  `docs/developer/g1-ghost-fidelity-journey.md` is that a balancing biped *must* deviate from a
  kinematic human reference, with a ~67 % physical wall against the raw human ghost.
- Training a model on data and distributing the model's outputs is an actively contested legal
  question, and here it is *further* mediated by a physics simulation.

**The case that they ARE:**

- The stated purpose of Shadowing is that the robot moves *like the reference* — we measure and
  publish WBMATCH scores in the 0.87–0.91 range against the ghost. Deliberate, measured,
  high-fidelity imitation of a specific recorded performance is exactly the fact pattern where an
  "it's a new work" defence is weakest.
- CC's "translated, altered, arranged, transformed, or otherwise modified" is broad, and does not
  obviously carve out "transformed by being used as a training target".
- The lineage is undisputed and self-documented — the files name their ancestor in their own
  metadata. We could not credibly claim independent creation.

**Why we deny anyway.** The owner's standard for this repository is *"completely legal and open
source, so no one can say anything otherwise."* An unsettled question does not meet that standard,
regardless of which way we privately think it would come out. And the deciding factor is cost:
**nothing that ships depends on any of these files** — verified across every
`projects/policies/skills/**/skill.json`, every world, every BATON sequence and every demo script.
Denying is free and reversible; being wrong in public is neither.

This is a release-gating decision, **not a finding that these files infringe anything.** If a
licence is later obtained (§8), un-denying them is a one-line revert.

Selected by **lineage, not by name**: 19 declare `from HOP-1` in their own `source`
(`ghost_v6`, `v7`, `v7rock`, `v8`, `v9a`, `v10`, `v12`, `v14`, `v15`, `v17`, `v18`, `v19`, `v20`,
`v20a`, `v21`, `v22`, `v22o`, `v61`, `v62`), and 4 more are hand-edits of those which carry HOP-1's
exact declared cadence `freq = 1.15625` (`ghost_v9`, `v11`, `v13`, `v16` — their sources read
"v10 straight-parallel arms", "v12 with MINIMUM rock", "v15 smooth gait").

> ⚠️ **Three lookalikes are deliberately NOT denied, and must not be "tidied" in later.** Their
> declared parent is the Unitree official motion or our own v3c, not HOP-1:
> `ghost_v5_lut.json` (*"from unitree official motion.pt recorded in-engine"*),
> `ghost_carried_lut.json` (same), and
> `ghost_walkstop_achieved_lut.json` (*"composed from ghost_v3c_lut.json"*).
> A name-based sweep of `ghost_v*` would wrongly catch all three plus the whole v3 family.

### Class 4 — the trained checkpoints (**12 files**)

**Ruling: DENIED, PRECAUTIONARY. Confidence: LOW on ND, MODERATE on NC.**

`projects/policies/training/runs/wr_{hop1_it100, v6calm_it150, v7trk_it150, v9b_it150, v9s2_it500,
v11_it200, v13_it250, v16_it200, v16c, smooth_it150, rhythm_champion, metronome_champion}.pt` — the
policies that produced Class 3, identified from the `policy=` field of each Class 3 LUT's `source`.

The ND argument is at its **weakest** here: a `.pt` is a tensor of network weights. No LAFAN1 frame
is extractable from it, and it is not motion data in any recognisable sense.

But the **NC argument is at its strongest**, and it is a different argument from everything above.
Section 2(a)(1) grants the Licensed Rights *"for NonCommercial purposes only"* — that restricts the
**use**, not merely the Sharing. Using LAFAN1 inside a training pipeline whose output is then
distributed under a commercial-permissive licence is a commercial purpose, and it sits outside the
grant whether or not the checkpoint is a "derivative work". Denying the checkpoints does not undo
the training run that already happened, but it does stop us distributing its product.

Cost of denial: **zero.** Two documents name these as historical champions in prose. Nothing loads
them.

> ⚠️ The flagship `wr_decent_walker.pt` is **not** in this set and must not be added. It trains
> against `ghost_official_full_v3_lut` (Class 1). Neither is `wr_showpiece.pt`,
> `wr_calm_champion.pt`, `wr_stepturn*`, `wr_turn*`, `wr_stair*`, `wr_stand*`, `wr_carry*`,
> `wr_backward.pt` or `wr_wsw2_it900.pt`.

### Class 5 — the parked dance campaign

**Ruling: already denied, and it stays denied — but note it is a DIFFERENT dataset.**

`projects/policies/ghosts/g1/dance/` (21 LUTs) is retargeted from BVH clips whose names
(`salsa_60_03.bvh`, `dance_93_04.bvh`, `box_143_11.bvh`, and an explicit `CMU 05_02`) are **CMU
Motion Capture Database** identifiers, not LAFAN1 — plus `charleston.bvh` and `lambada.bvh`, whose
origin is **not recorded anywhere in the tree**. The CMU database is distributed on notably
permissive terms, so some of this directory might well be shippable, but:

1. we have no record of which clip came from where,
2. `charleston.bvh` and `lambada.bvh` have no provenance at all, and
3. the campaign is parked, so nothing depends on the answer.

**UNRESOLVED, conservative action taken.** What would settle it: a per-clip provenance record
naming the source database and its terms for each of the six BVH sources. Until then the directory
stays denied as a whole — not because CMU is restrictive, but because we cannot prove which clips
are CMU.

---

## 5. What we did NOT deny, deliberately

- **`projects/policies/training/lafan1_to_ghost.py`.** Our own code, Apache-2.0, containing no
  LAFAN1 sample — it takes the clip path as a command-line argument. Upstream draws the same
  code/data line (*"unlike the code, which is licensed under MIT"*). A reader who holds their own
  LAFAN1 copy under Ubisoft's terms can still use it. Denying a tool because of the format it reads
  would be over-broad.
- **Prose mentions.** `g1_walk_recipe.py`, `ghost_doctor.py`, `ghost-design-rules.md`,
  `policy-switching.md` and `step-turn-method.md` refer to "the hum70/hum40 lesson" or "raw LAFAN1
  pace is infeasible". Naming a dataset and recording what we learned from it is not redistributing
  it, and these are some of the most useful lessons in the tree.
- **Class 0 and Class 1 ghosts**, per the table above.

---

## 6. Still open: the Unitree `motion.pt` lineage (Class 1)

Flagged here because this audit surfaced it, and because it is now the **largest remaining
unverified third-party input** to the ghost store — including the flagship walk reference.

Class 1 ghosts are recordings of Unitree's released G1/H1 walking policy replayed in OmniSim. We
have not verified what licence that policy ships under, nor whether recording a robot's motion while
running someone else's released policy produces anything the policy's author has rights in.

Our tentative read is that this is much weaker than the LAFAN1 case — the artifact is a physics
simulation of *our* robot model, the policy is released for exactly this kind of use, and network
weights driving a simulation are further from an authored work than mocap of a human performance is.
But *tentative read* is precisely what §2 shows to be dangerous, and it is what produced this leak
in the first place.

**Action required (not owned by this document):** verify the licence on `unitree_rl_gym` /
whichever Unitree release `motion.pt` came from, and record the result here. Until then, do not
describe the flagship G1 walk's provenance as "verified clean" — describe it as "Unitree lineage,
terms not yet verified".

---

## 7. Files denied by this ruling

48 files newly excluded from the public snapshot on 2026-08-22, added to
`scripts/release/publish_deny.txt` with the reasoning inline:

| class | count | what |
|---|---|---|
| 2 (transitive) | 13 | `ghost_hop1*_lut.json` (12) + `ghost_medley1_lut.json` (1) |
| 3 | 23 | second-order re-records from HOP-1 |
| 4 | 12 | the checkpoints that produced them |

Plus the 11 pathspec entries already denied on 2026-07-11 (9 LUT files, the raw-data directory,
the two builder scripts, the mocap-skeleton controller, a runner and two research worlds).

**Cascade: 7 lines across 3 surviving files.** The snapshot's dangling-reference check compares the
last two path segments of every removed file against everything that ships; simulated against this
denial set, it reports:

| surviving file | lines | reference |
|---|---|---|
| `docs/developer/rl-current-state.md` | 254, 258 | `runs/wr_v13_it250.pt`, `runs/wr_v11_it200.pt` — prose naming historical champions |
| `projects/policies/training/README.md` | 72, 73, 77 | a copy-pasteable training command using `ghost_hop1v11_lut.json`, `ghost_v14_lut.json`, `wr_v13_it250.pt` |
| `projects/policies/training/ghost_doctor.py` | 227, 228 | a default comparison pair, `ghost_v62_lut.json` + `ghost_v8_lut.json` |

**That number is the good news, and it is worth stating plainly: this material is shallow, not
embedded.** Zero skill manifests, zero worlds, zero demos, zero BATON sequences and zero deploy
scripts touch any of it. A dataset that had worked its way into the shipped demos would have made
this a migration; instead it is a deny-list edit plus three cosmetic reference fixes.

The check is advisory by default (`PUBLISH_STRICT_DANGLING=0`), so the release will not break — but
the three files above should be updated by their owners, because a public reader following that
training command will hit a file that does not exist. Recommended fixes:

- `projects/policies/training/README.md` — retarget the example command at lines 72–77 onto a Class 0
  or Class 1 pair (e.g. `GHOST_LUT_JSON=ghost_official_full_v3_lut.json` with the flagship
  `wr_decent_walker.pt`), which is the current recipe anyway.
- `projects/policies/training/ghost_doctor.py` — change the default comparison pair at lines 227–228
  to two shipping LUTs.
- `docs/developer/rl-current-state.md` — lines 254/258 are historical prose; either leave them (a
  named-but-absent checkpoint in a history section is harmless) or add "(not in the public
  snapshot)" as that document already does elsewhere.

---

## 8. What would change this ruling

For Classes 2–4 to be un-denied, one of:

1. **A licence grant from Ubisoft La Forge** permitting redistribution of derivatives on
   commercial-permissive terms. This is the only thing that clears Class 2 outright.
2. **Rebuilding the references without LAFAN1.** Entirely feasible and probably the right long-term
   move: `ghost_synth*.py` is ours and already produces validator-passing walk and turn ghosts, and
   HOP-1's non-LAFAN1 inputs (Unitree official + our v3c) are both available. A regenerated HOP-1
   equivalent with the LAFAN1 term dropped or replaced would let the whole Class 3/4 lineage ship,
   at the cost of retraining.
3. **A reasoned decision by the owner to accept the Class 3/4 risk** on the argument in §4 — that a
   physics re-recording of a robot is not a derivative of the mocap that influenced its training
   reward. That is a defensible position; it is just not a *provable* one, which is why it is not the
   default here.

Nothing about Class 2 changes without (1) or (2). Class 2 is not a judgement call.

---

## 9. Sources

| what | URL | fetched |
|---|---|---|
| LAFAN1 licence text (CC BY-NC-ND 4.0) | <https://github.com/ubisoft/ubisoft-laforge-animation-dataset/blob/master/license.txt> | 2026-08-22 |
| LAFAN1 README licence statement | <https://github.com/ubisoft/ubisoft-laforge-animation-dataset> | 2026-08-22 |
| Retarget mirror we used (ungated, no `license:` tag) | <https://huggingface.co/datasets/lvhaidong/LAFAN1_Retargeting_Dataset> | 2026-08-22 |
| Stated upstream retarget (gated — HTTP 401) | <https://huggingface.co/datasets/unitreerobotics/LAFAN1_Retargeting_Dataset> | 2026-08-22 |
| CC BY-NC-ND 4.0 canonical text | <https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode> | — |

If you use LAFAN1 in research, its authors ask to be cited:

```
@article{harvey2020robust,
author    = {Félix G. Harvey and Mike Yurick and Derek Nowrouzezahrai and Christopher Pal},
title     = {Robust Motion In-Betweening},
...
}
```

---

## 10. The process lesson

The 2026-07-11 deny-list entry for this material said, in its own words, that *"the redistribution
terms were never verified"* — and then that sentence sat there, unchanged, while the files stayed
held and nobody verified anything. Verification took about twenty minutes: the licence was a
`license.txt` in the root of a public GitHub repository the whole time.

Two rules worth keeping:

1. **A deny-list entry that says "unverified" is a TODO, not a resolution.** It stops the bleeding;
   it does not close the wound. Holding a file forever because nobody checked is its own failure
   mode — three of the ghosts denied here may well have been shippable all along, and we will never
   know for the dance directory because the provenance was never written down.
2. **Provenance must live where it ships.** The only record of this problem was in `SOURCE.md`,
   which is deny-listed, in a repository whose release squashes git history. The problem was
   therefore invisible to every public reader *and* to every future agent working from the public
   tree. That is why this document exists and why it must keep shipping.
