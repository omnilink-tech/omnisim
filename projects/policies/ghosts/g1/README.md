# G1 ghost store — lineage and licence

Every file here is a **ghost**: a phase-indexed lookup table of joint angles that a Shadowing policy
tracks. Each one declares where its numbers came from in its own `source` field, and **that field is
load-bearing** — it is what the licence ruling below is derived from, and it is why you must never
hand-edit a LUT's provenance to say something its numbers do not support.

**Ruling made 2026-08-22. Full reasoning, licence text and URLs:
[`docs/developer/motion-data-provenance.md`](../../../../docs/developer/motion-data-provenance.md).**
Read that before adding a ghost with any third-party ancestry, or before un-holding anything.

---

## The four lineage classes

Every G1 ghost belongs to exactly one. Check the `source` field, not the filename — the naming is
historical and a `ghost_v*` name says nothing about ancestry.

### Class 0 — synthesised / planned from scratch — **ships, unrestricted**

Built by our own tooling: `ghost_synth_walk.py`, `ghost_synth_kneel.py`, `ghost_synth_pushup.py`,
`ghost_synth_squat.py`, `ghost_synth.py` (stairs), `build_step_turn_ghost.py`, `ghost_topp.py`, and
hand-authored FK/IK poses. We plan the contacts and solve for everything else, so the numbers are
ours outright and carry the repository's Apache-2.0 licence like any other source file.

`ghost_walk_synth`, `ghost_walk_fast_v1`, `ghost_walk_backward64`, `ghost_carry_*`, `ghost_crawl_v1`,
`ghost_kneel_synth`, `ghost_pushup_synth`, `ghost_squat_synth`, `ghost_stair*`, `ghost_walk_climb`,
`ghost_step_turn*`, `ghost_turn_*`, `ghost_stand_v1`, `ghost_v3c`–`ghost_v3g`, `ghost_walkstop`.

**If you need a new reference and have a choice, build it here.** It is the only class with no
provenance question at all.

### Class 1 — recorded from the Unitree official policy — **ships; terms not yet verified**

Unitree's released `motion.pt` walking policy replayed inside OmniSim, with the robot's own joint
sensors sampled. No motion-capture data of any kind is involved.

`ghost_unitree{,_u115,_u130}`, `ghost_official_arms`, `ghost_official_metric`,
`ghost_official_full{,_v2,_v3}`, `ghost_walk_mid_v1`, and the re-records whose declared parent is the
official motion: `ghost_v5`, `ghost_carried`.

> `ghost_official_full_v3_lut.json` is the **flagship** — the reference `skills/humanoid/g1_walk`
> and every BATON sequence ride on.
>
> ⚠️ **Open item.** We have *not* verified the licence on Unitree's released policy, so do not
> describe this class as "verified clean". Describe it as "Unitree lineage, terms not yet verified".
> See §6 of the provenance document.

### Class 2 — LAFAN1-derived — ⛔ **HELD, does not ship**

Ubisoft La Forge's LAFAN1 mocap, community-retargeted to the G1, blended into our references.

**Licence: Creative Commons Attribution-NonCommercial-NoDerivatives 4.0.** Its grant, verbatim, is to
*"produce and reproduce, **but not Share**, Adapted Material for NonCommercial purposes only"*, and
it adds: *"For the avoidance of doubt, You do not have permission under this Public License to Share
Adapted Material."* A retarget or a blend **is** Adapted Material. Two independent blockers:
**ND** withholds the right to redistribute derivatives at all, and **NC** is incompatible with
Apache-2.0, which promises every recipient the right to use and sell.

Attribution does not fix this. ND is a missing grant, not an attribution condition — there is no
notice you can add that buys back a right the licence never gave.

Held: `ghost_lafan1_*`, `ghost_hum{40,70,100,edge}`, `ghost_pivot_turn*`, all of `dance/`, and —
added 2026-08-22, because they were leaking — every `ghost_hop1*` (each declares
`unitree->hum40 at alpha=0.375`, i.e. ~15 % LAFAN1 by value) and `ghost_medley1` (assembled from
LAFAN1 clip names).

**Confidence: high.** This one is not a judgement call.

### Class 3 — second-order re-records from a Class 2 reference — ⛔ **HELD, precautionary**

`RE-RECORDED achieved motion (rule 4): policy=<checkpoint> ... from HOP-1 center`.

These are the **robot's own motion**, sampled in the physics engine, produced by a policy that had a
HOP-1 LUT in its training reward. No LAFAN1 sample value survives — only the influence of a training
target, through a lossy non-invertible physical process. Whether that makes them derivative works is
**genuinely unsettled**, and holding them is a release-gating decision, **not a finding that they
infringe anything**. It is taken because an unsettled question does not meet this project's standard,
and because the cost is zero: nothing that ships depends on any of them.

Held: `ghost_v6`, `v7`, `v7rock`, `v8`, `v9`, `v9a`, `v10`, `v11`, `v12`, `v13`, `v14`, `v15`, `v16`,
`v17`, `v18`, `v19`, `v20`, `v20a`, `v21`, `v22`, `v22o`, `v61`, `v62`, plus the checkpoints under
`training/runs/` that produced them.

> ⚠️ **Three lookalikes are deliberately NOT held. Do not "tidy" them in.**
> `ghost_v5` and `ghost_carried` declare *"from unitree official motion.pt recorded in-engine"*;
> `ghost_walkstop_achieved` declares *"composed from ghost_v3c_lut.json"*. They are Class 1 and
> Class 0 respectively. A name-based sweep of `ghost_v*` would wrongly catch all three plus the
> entire v3 family — which is exactly why the cut is made on `source`, not on filenames.

---

## Adding a ghost

Beyond the validation steps in [`../README.md`](../README.md), one provenance rule:

**Write the real ancestry into `source`, including any third-party input, at the moment you build
the file.** Name the dataset, the clip, the frame range and the blend weight. If a later reader
cannot reconstruct the lineage from the artifact itself, the artifact is unshippable by default —
that is not a hypothetical, it is what happened to `dance/`, whose `charleston.bvh` and `lambada.bvh`
have no recorded origin anywhere in the tree and which therefore stays held even though its other
clips are probably permissively licensed.
