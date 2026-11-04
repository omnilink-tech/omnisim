# Husky slope/friction/payload assessment — 2026-10-28

This is a structured simulation discovery campaign with **zero braking delay**.
The [full report](report.html), [campaign record](campaign.json),
[outcome map](../../../../docs/media/assessments/husky-envelope.png), and
[matched-friction comparison](../../../../docs/media/assessments/husky-stop-comparison.png)
are derived from the retained per-step traces.

## Conditions

- Existing Husky URDF with a dedicated wheel-velocity test controller; no vendor
  firmware, learned policy, or physical hardware in this experiment.
- Target approach speed 0.6 m/s; stable measured speed and at least 0.3 m travel
  required before STOP. Immediate brake application in all qualified cases.
- Experimental requirement: forward excursion at most 0.25 m, settle below
  0.025 m/s within 1.5 s, remain settled through the observation. Samples every
  16 ms, at least 2.5 s observed after STOP, 8 physics substeps.
- Downhill slopes 0/5/10/20°, tyre/surface contact μ 0.10/0.20/0.40/0.80, payload
  0/10/20 kg. Cargo is a rigidly mounted 0.30 × 0.25 × 0.20 m box, center 0.35 m
  above the chassis frame at x=y=0; not loose cargo or a manufacturer load rating.
- Newton 1.5.0 / MuJoCo 3.11.0 CPU, Windows 11, machine `9722d23d12a3`, AMD64
  Family 25 Model 80, 16 logical cores. The per-case fingerprints contain full
  runtime versions and engine/controller hashes. These measurements are not a
  simulator throughput or comparative-cost benchmark.

## Results and boundary brackets

All 64 planned cases completed with valid adopted fixtures and matching offline
regrades: 48 grid cases, 12 local refinement samples and four fresh confirmations.
41 cases passed and 23 failed. These counts describe the deliberately selected
test set, **not a deployment failure probability**.

The coarse 48-case grid contains 30 passes, nine qualified braking failures and
nine initial-hold/approach failures. The 0/10/20 kg loads produced the same coarse
pass/fail pattern, with small changes in measured travel. No maximum payload
capability was established.

At **5° downhill, 10 kg payload**:

| Case | Contact μ | Result | Maximum observed travel after STOP |
|---|---:|---|---:|
| [case-013](case-013/verdict.json) | 0.10 | FAIL, distance and settling deadline | 1.125979 m; still moving at 0.295452 m/s at end |
| [case-016](case-016/verdict.json) | 0.20 | PASS | 0.166145 m; settled after 0.528 s |

The second case changes the imposed contact coefficient while retaining slope,
load, speed target and immediate braking. An observed displacement in a case
that never settled is not its eventual stopping distance.

After refinement, the 10 kg qualified-braking transitions are bracketed by:

| Downhill slope | Sampled FAIL coefficient | Sampled PASS coefficient |
|---|---:|---:|
| 5° | 0.10 | 0.20 |
| 10° | 0.25 | 0.30 |
| 20° | 0.40 | 0.50 |

These are sampled local transitions at the stated task settings, not exact
critical coefficients or a globally certified safe envelope. Unsampled conditions,
other speeds, mounting positions, surfaces, firmware, and hardware can differ.

Four fresh-engine confirmations repeated the unloaded 5° FAIL/PASS pair
([cases 060–063](campaign.json)). All outcomes matched; the measured travel
difference from each parent case was 0 in these repeats. First confirmations
retain their own native request/final stills. This is an observation for these
runs, not a universal determinism claim.

## Physical validity and provenance

`adoption.json` checks the compiled solver's total mass, floor quaternion, and
friction on floor **and robot geoms**. Husky's original tyre Gazebo tags override
the world coefficient; the generated model therefore assigns the intended
contact coefficient to those tags as well. Original vendor files are unchanged.
The importer contributes its documented 0.1 kg root mass; total compiled mass is
checked against the original declared masses plus that root contribution and
the payload. A bad fixture is ERROR, not FAIL.

All raw attempts, logs, compiled model dumps, commands, traces, completion records,
fingerprints, specs and verdicts are retained. `source/` preserves the assessment
implementation used during the campaign. Later presentation-only report revisions
are recorded separately; raw evidence is not rewritten. Exact reruns require the
matching source/model/runtime hashes. The guide documents how to generate a new
campaign with the current checkout.

After correcting presentation-only generic-robot wording in the HTML renderer,
the current checkout also executed a new 5°/10 kg/μ=0.10 failure and its exact
fresh reproduction. The matching traces and current source hashes are in
[`husky-current-natural-failure-20261028`](../husky-current-natural-failure-20261028/)
and [`husky-current-natural-reproduction-20261028`](../husky-current-natural-reproduction-20261028/).
Both reproduce case-013's 1.125979 m observed excursion with zero brake delay.

```powershell
python -m omnisim assess sweep --robots husky --refine 12 --confirm 2 --workers 2 --out .tmp/my-new-husky-envelope
```
