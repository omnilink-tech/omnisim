# OmniSim Agent Build Films

This is the locked workflow for videos in which an AI agent shows what it built
in OmniSim. It generalizes the approved **Agent Build Story v6** film without
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

### 3. Create the film manifest

```powershell
python -m omnisim cinema agent-build-new --title "What I built with OmniQuad" > agent_build.json
```

Replace every placeholder with the build’s real question, bounded claim,
boundary, captures, source ranges, shot purposes, information cards, and voice
windows. The starter manifest is also committed at
[`storyboards/agent_build_template.json`](storyboards/agent_build_template.json).

Validate before spending time on voice or rendering:

```powershell
python -m omnisim cinema agent-build-validate agent_build.json
```

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

### 5. Assemble and verify

```powershell
python -m omnisim cinema agent-build-render agent_build.json
python -m omnisim cinema agent-build-verify agent_build.json
```

The renderer creates the locked intro/outro, trims and grades real captures,
renders safe cards and motion plates, generates the restrained score, ducks the
score under narration, and emits:

- the 1080p30 H.264/AAC master;
- a complete edit-decision list;
- capture hashes and approved source ranges;
- picture, narration, score, manifest, EDL, provenance, and master hashes in an
  assembly receipt;
- a machine-readable verification report.

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

## What belongs in Git

Commit manifests, narration text, capture directions, worlds/controllers needed
to reproduce the build, evidence JSON, review records, and pipeline changes.
Generated voice files, captures, render caches, scores, and masters remain local
or are published through the appropriate media channel. The assembly receipt
binds local deliverables without making heavy render products source files.

## Approved reference

The style is defined by
[`social/education/build_with_omnisim/AGENT_BUILD_STORY_MILESTONE.md`](../../social/education/build_with_omnisim/AGENT_BUILD_STORY_MILESTONE.md).
The approved local v6 master has SHA-256
`0b26a58ae830a904fccd57cf47b521696b2671de84db8032961c0c121dac0678`.
