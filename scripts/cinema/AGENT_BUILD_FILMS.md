# OmniSim Agent Build Films

This is the locked workflow for videos in which an AI agent shows what it built
in OmniSim. It generalizes the approved **Agent Build Story v8** film without
loosening its visual, editorial, or evidence standards.

The capture service and the ordinary Cinema storyboard system create real
OmniSim footage. The Agent Build Film system is the story/edit layer above
those captures.

## The immutable signature

Every film begins and ends the same way.

| Time | Picture | Voiceover |
|---|---|---|
| 0:00–0:05 | Designed disclosure: “This video was made end-to-end by an AI agent under human monitoring.” | None |
| 0:05–0:10 | Pure black screen: “A real build.” / “The story of an agent.” / “Told by an agent.” | None |
| 0:10 | First real build shot | First narration line |
| Final 4.5 s | OmniSim orb, “FROM EDIT TO EVIDENCE”, and `github.com/omnilink-tech/omnisim` | None by default |

The exact ranges, text, silence, GitHub destination, and first-footage time are
code-enforced. They are not per-film options.

## The story contract

Duration is **story-driven**. There is no target runtime and no upper runtime
limit. A film is as short as its evidence allows and as long as its coherent
single-build story requires. Do not pad to a platform target and do not cut an
essential attempt, control, or explanation merely to hit a number.

The human supplies the build idea and monitoring. The AI agent owns the build
implementation, experiment design, troubleshooting, evidence, narration, and
edit, and tells that first-person story honestly. A build brief is not a script.

The primary picture is always the simulator itself. A three-act film must keep
at least **75 percent of its post-intro story duration in real OmniSim clips**;
the manifest validator measures and enforces that ratio. Diagrams and full-screen
plates are short orientation tools, not the visual default. Explain world design,
control, failure, and evidence over the most relevant simulator footage whenever
the footage can carry the idea.

The default narrative form is a three-act journey:

1. **Act I — setup:** reveal the world, introduce the robot, and establish one
   question plus the cost of failure.
2. **Act II — confrontation:** show the attempt, negative control, engineering
   obstacle, and the specific edit that earns another run.
3. **Act III — resolution:** stay with one continuous decisive run through the
   climax, show the result, state the boundary, and resolve the build.

The climax is a real Act III clip, never a diagram, and it must be at least 12
seconds. For a journey build, the viewer should be able to follow the robot from
the beginning of the decisive attempt through the result without a montage of
unrelated successes.

An Agent Build Film is not a capability montage. It must contain these beats,
in this order:

1. **Question** — begin on real footage of the robot, object, or environment and
   state one measurable robotics question.
2. **Attempt** — show the build appearing to work in a frame where the result is
   judgeable.
3. **Control** — change one declared variable and produce a controlled failure,
   counterexample, or negative control.
4. **Evidence** — compare the outcomes and state only the claim the comparison
   earns.
5. **Method** — explain how the agent described, inspected, edited, ran,
   measured, and refined the build.
6. **Transfer** *(optional)* — apply the same method to another robot only when
   it advances the story.
7. **Boundary** — disclose the support condition, simulation limit, or untested
   assumption on the first relevant frame.
8. **Conclusion** — resolve on the reusable method, not a list of features.

The manifest validator rejects a missing or out-of-order required beat.

## Camera and edit rules

- Frame the task before recording. Use `POST /scene/frame` or a subject-relative
  Cinema primitive; do not discover framing through decorative camera motion.
- Lock the camera by default. Orbit, rotation, translation, and zoom are allowed
  only when they reveal new spatial information needed to judge the build.
- Never attach a chase camera to a heavily time-compressed robot. Use locked
  establishing shots and locked regional coverage; when the robot leaves a
  region, make a motivated direct cut to the next pre-framed region. The subject
  moves quickly, but the camera does not whip after it.
- Use a spatial-orientation ladder. Establish the whole workspace in Act I,
  return to a wide or bird's-eye reference during the decisive Act III action,
  and then cut back to detail. Never run more than two uninterrupted detail
  clips without a wide or medium reorientation. A wide shot is evidence: it
  lets the viewer judge route, workspace, separation, and remaining distance.
- A decisive-run wide must contain the actual subject moving through the
  simulator. A static overview, map reconstruction, or live inset does not
  satisfy the Act III spatial-reference requirement. If the subject becomes
  too small at whole-workspace scale, a restrained film-only locator may follow
  the robot visible in the recorded frames or a recorded measured pose. Its
  provenance must say how it was driven and that it changes no physics,
  sensing, planning, or result.
- Build the Act III scale ladder deliberately: judgeable detail, moving
  whole-workspace context, final-detail approach, unobstructed result. The last
  build frame before the outro must show the achieved outcome clearly; do not
  resolve on an occluded robot, an empty corridor, or a generic establishing
  shot.
- Every shot has a written `purpose`. Every cut is a direct cut and marks a real
  change in question, evidence, scale, or point of view.
- Keep action chronological. A rewind is legal only as an on-screen labeled
  **ANALYTICAL REPLAY**.
- Use real OmniSim captures as evidence. Do not substitute game-like generated
  imagery, inert footage, or a visual reconstruction for a simulation result.
- Information cards stay inside the 72 px safe area and never cover the action.
  Full-screen plates are used when the information is too dense for a card.
- Claim limits stay visible from the first frame where the claim becomes
  relevant. A weight-bearing harness, model-based controller, or simulation-only
  result is part of the story, not a footnote.
- No subtitle stream, persistent corner logo, arbitrary transition, or unowned
  URL is permitted.

## Audio hierarchy

The score supports the film; it never competes with the disclosure, narration,
or simulated action. The locked mix uses a deliberately softer gain during the
silent 0–10 second intro, a restrained story bed afterward, and side-chain
ducking beneath speech. Do not raise the bed to compensate for a quiet voice;
fix voice level or performance at its source.

Measure the delivered master, not only the pre-normalized score. The average
0–10 second intro level must be at least **3 dB below** a representative narrated
story window after the final loudness stage. The reference maze benchmark reads
−23.0 dB mean for the intro versus −19.0 dB for Act I.

## Simulation-fidelity release gate

Delivery resolution and simulation rendering quality are separate gates. An
upscaled flat render does not pass. For hero footage, prefer the **wgpu main
view** with its HDR/AgX, atmosphere, PBR, shadow, AO, reflection, TAA, and
OmniLight stack. The wgpu Camera-device path is valid sensor evidence but has a
deliberately different display contract; do not silently substitute it for the
main-view beauty look.

Author the world for the renderer: textured/normal-mapped PBR surfaces, one
motivated shadow-casting sun, controlled fill, readable exposure, and physical
scale. Warm the renderer before capture. Capture a short proxy first and inspect
opening/middle/closing frames for subject visibility, occlusion, exposure,
material response, and camera purpose. Only then capture the final sequence.

Main-view frame sequences are verified independently from the final edit:

```powershell
python scripts/cinema/sim_quality.py verify `
  --frames captures/shot_frames --log evidence/shot.log `
  --world worlds/build.omniworld --out evidence/sim_quality.json
python scripts/cinema/sim_quality.py encode `
  --frames captures/shot_frames --output captures/shot_wgpu.mp4 --crf 12
```

The encoder writes a sampled-frame/hash receipt next to the clip and reuses the
clip when the frame spool and settings still match. A repeated edit therefore
does not spend another full PNG decode and high-quality encode pass.

The gate proves the wgpu/AgX runtime markers, PBR/normal-map/shadow authoring,
stable resolution, and non-degenerate exposure/detail. Creative frame review is
still required because statistics cannot prove that the robot is visible.

## Efficient production contract

- Freeze the behavioral controller and collect headless success/control
  receipts before rendering.
- Use a low-cost, short proxy and review representative frames before any full
  capture.
- Time-compress long simulations at capture cadence; preserve the uncompressed
  controller receipt as the behavioral source of truth.
- Write final frame sequences into content-addressed or uniquely named folders.
  If the world, controller, camera profile, and frame hashes match, reuse them.
- Record one continuous build state when possible and take only story-relevant
  ranges into the edit. Do not repeatedly rerender unchanged evidence.
- Render voice and motion plates only after the source ranges and story order
  validate. Render the final master once, then rerender only failed segments.

## End-to-end workflow

Run commands from the repository root.

### 1. Establish the simulation truth

```powershell
python -m omnisim doctor
```

Do not capture if the binary or Newton physics runtime is missing. Build and run
the smallest test that can answer the question. Record the successful run and a
single-variable control. Preserve controller logs, result JSON, world/controller
hashes, and the exact environment variables that distinguish the runs.

### 2. Capture purposeful shots

Use the capture service directly for a known shot list, or the subject-relative
Cinema layer when it helps:

```powershell
python -m omnisim capture
python -m omnisim cinema render my_capture_storyboard.json --no-edit
```

Review the captured frames before editing. Reject empty, intersecting, inert,
misframed, ambiguous, or unsupported footage.

For the high-fidelity wgpu main-view profile, also run the simulation-fidelity
gate above before the clip is admitted to the manifest.

### 3. Create the film manifest

```powershell
python -m omnisim cinema agent-build-new --title "What I built with OmniQuad" > agent_build.json
```

Replace every placeholder with the build’s real question, bounded claim,
boundary, captures, source ranges, shot purposes, information cards, and voice
windows. The starter manifest is also committed at
[`storyboards/agent_build_template.json`](storyboards/agent_build_template.json).

The starter manifest also carries the enforced simulator-first structure:

```json
"editorial": {
  "structure": "three_act",
  "simulator_footage_ratio_min": 0.75,
  "climax_segment": "climax",
  "spatial_reorientation_required": true,
  "max_consecutive_detail": 2,
  "wide_reference_segments": ["build_question", "climax"]
}
```

Every segment in that form declares `"act": 1`, `2`, or `3`; acts cannot run
backward, and `climax_segment` must name an Act III simulator clip.

Validate before spending time on voice or rendering:

```powershell
python -m omnisim cinema agent-build-validate agent_build.json
python -m omnisim cinema agent-build-preflight agent_build.json
```

The preflight opens each unique capture once, rejects impossible source ranges,
checks every evidence file, and estimates every narration block before the TTS
model or encoder is loaded. A deliberate held final frame must be declared with
`"source_tail_hold_s"` (maximum three seconds); silent accidental padding is
never accepted.

### 4. Write and generate the voice

Write one short narration paragraph per manifest voice window. The first block
begins at exactly second 10. Use first person only for actions the agent actually
performed. Do not narrate either intro screen.

Set up the checksum-verified local voice runtime once:

```powershell
python -m omnisim cinema agent-build-voice-setup
```

Generate the natural performance:

```powershell
python -m omnisim cinema agent-build-voice agent_build.json
```

The generator uses calm first-person delivery, authored clause/sentence pauses,
and a natural speed near 0.96–1.00. If copy exceeds its window, generation fails
and the copy must be shortened; the workflow does not force an unnatural pace.

### 5. Proxy, critique, assemble, and verify

```powershell
python -m omnisim cinema agent-build-make agent_build.json
```

This is equivalent to the resumable sequence below:

```powershell
python -m omnisim cinema agent-build-voice agent_build.json
python -m omnisim cinema agent-build-proxy agent_build.json
python -m omnisim cinema agent-build-review agent_build.json
python -m omnisim cinema agent-build-render agent_build.json
python -m omnisim cinema agent-build-verify agent_build.json
```

The release render is not reached by `agent-build-make` until the exact proxy
passes. The local critique measures story-critical movement (including a small
subject in the Act III wide), excessive full-frame movement in wides, exposure,
frame detail, output frame counts, overlay competition, cut-boundary integrity,
and the last build frame. It also writes ordinary overview and exact cut-pair
sheets. Pixel tests cannot establish the semantic truth of a goal or a claim,
so the independent monitored review below remains mandatory.

The renderer creates the locked intro/outro, trims and grades real captures,
renders safe cards and motion plates, generates the restrained score, ducks the
score under narration, and emits:

- the 1080p30 H.264/AAC master;
- a complete edit-decision list;
- capture hashes and approved source ranges;
- picture, narration, score, manifest, EDL, provenance, and master hashes in an
  assembly receipt;
- a machine-readable verification report.

All expensive artifacts are content-addressed. A changed narration paragraph
regenerates only that voice block; a changed shot regenerates only that part;
audio-only work never rebuilds the picture; unchanged intro/outro, plates,
score, proxy, and final mix are reused. Every run writes cache hits, misses, and
stage timings to `workflow_performance.json`.

### Production-time contract

- Ready, working simulation: **35 minutes or less** for a film at benchmark
  scale.
- Normal script, mix, or single-shot revision: **five minutes or less**.
- Novel controller, physics, or world engineering is measured separately and
  is never hidden inside the filmmaking benchmark.

For new captures, create a plan and keep one capture service alive:

```powershell
python -m omnisim capture
python -m omnisim cinema agent-build-capture-new --world worlds/build.omniworld > capture_plan.json
python -m omnisim cinema agent-build-make agent_build.json --capture-plan capture_plan.json
```

The plan loads its world once, creates named scene checkpoints, captures the
shot list, persists a receipt after every shot, and skips hash-identical shots
on resume. Scene checkpoints do not rewind arbitrary controller-process memory;
the receipt states that boundary. Reload the world when controller-memory
identity is part of the claim.
The committed starting point is
[`storyboards/agent_build_capture_template.json`](storyboards/agent_build_capture_template.json).

The release gate rejects intro timing drift, voice activity before second 10,
source rewinds without replay disclosure, missing story beats, stale hashes,
wrong frame count, subtitles, codec drift, or an outro other than the OmniSim
GitHub slate.

### 6. Independent release review

Automated approval is necessary but not sufficient. Before publication, obtain:

1. a creative/story review of every cut and frame;
2. a factual review of every simulation and authorship claim;
3. a fail-closed systems review of timing, media, hashes, and receipts.

If any review finds a blocker, fix the source or guardrail, rebuild, and rerun all
three reviews. Do not waive a blocker because the video looks good.

The creative review is performed twice: first on regular overview sheets, then
on exact before/after cut pairs. It must explicitly inspect the first frame at
10 seconds, the negative-control failure, every information-card safe area, the
Act III detail→wide→detail ladder, the visible moving subject in the wide, the
goal/result frame, the claim boundary, and the last build frame before the
outro. A regular contact sheet alone can miss a bad cut or obstructed ending.

## What belongs in Git

Commit manifests, narration text, capture directions, worlds/controllers needed
to reproduce the build, evidence JSON, review records, and pipeline changes.
Generated voice files, captures, render caches, scores, and masters remain local
or are published through the appropriate media channel. The assembly receipt
binds local deliverables without making heavy render products source files.

## Approved references

The foundational style is defined by
[`social/education/build_with_omnisim/AGENT_BUILD_STORY_MILESTONE.md`](../../social/education/build_with_omnisim/AGENT_BUILD_STORY_MILESTONE.md).
The approved local v6 master has SHA-256
`0b26a58ae830a904fccd57cf47b521696b2671de84db8032961c0c121dac0678`.

The current benchmark for single-build story structure, spatial continuity,
critique, and audio hierarchy is
[`social/education/agent_build_unseen_maze/agent_build_benchmark.json`](../../social/education/agent_build_unseen_maze/agent_build_benchmark.json).
Its approved 146.866-second local master has SHA-256
`76542458d329683ef616f6635d60390e42954addbd5d5bbaf215dd1fd905b57b`.
The binding independent review is
[`reviews/benchmark_release_review.json`](../../social/education/agent_build_unseen_maze/reviews/benchmark_release_review.json).
