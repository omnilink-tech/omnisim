# scripts/cinema/ — agent-driven cinematic pipeline

**OmniLink command demos** use the owner-approved
[minimal native style](OMNILINK_DEMOS.md): one robot, one arena, silence,
prompt → action. Use that guide for OmniLink demos; the general cinematic
workflow below applies to other productions.

New cinematic productions use **OmniSim simulation → recorded poses → authored
Blender scene → Cycles proxy → review → final render → edit**. This separates
the physics from the cinematic appearance while preserving the measured motion.
Read the canonical [cinematic replay workflow](CINEMATIC_REPLAY.md) first.

The package also retains the explicit native capture director and the Agent
Build editing tools. Native captures are useful when the live simulator image
itself is the evidence; they are not the default beauty-rendering route.

Entry point: **`python -m omnisim cinema <subcommand>`**.

## The 60-second tour

```bash
python -m omnisim cinema new --title "My robot's task" --world world.omniworld > film.json
# Prepare the recorded motion, run evidence, packed scene and shot list.
python -m omnisim cinema replay-validate film.json
python -m omnisim cinema render film.json --device OPTIX
# Inspect the moving proxy clips and contact sheet; record actual findings.
python -m omnisim cinema replay-review film.json --notes "Your review findings"
python -m omnisim cinema render film.json --profile final --device OPTIX
```

Blender and ffmpeg are local prerequisites. CPU is the portable default device.
The generated manifest resolves paths beside itself and writes under `renders/`.
Final rendering requires a review tied to the current source files and proxy.
The [full guide](CINEMATIC_REPLAY.md) covers recording, binding, scene preparation,
quality settings, evidence, output artifacts and limitations.

## Explicit native capture tour

1. Start the native capture service once and leave it running:

   ```bash
   python -m omnisim capture --port 6791 &
   ```

2. Get a starter storyboard:

   ```bash
   python -m omnisim cinema new --renderer native --title "OmniQuad RL hero" --subject omniquad \
     --world projects/policies/research/worlds/omniquad_rl_deploy.omniworld > omniquad_hero.json
   ```

3. Render it end-to-end:

   ```bash
   python -m omnisim cinema render omniquad_hero.json
   ```

   Output lands in `social/youtube_videos/captures/cinema_<slug>_<ts>/`:

   - `<slug>_16x9.mp4`, `<slug>_9x16.mp4`, `<slug>_1x1.mp4` — the
     branded deliverables, one per aspect ratio in the storyboard
   - `shots/sNN_<beat>_<primitive>.mp4` — the raw per-shot clips
     (rendered before grade + assembly, so you can re-edit later)
   - `manifest.json` — every keyframe, every choice, fully reproducible

## Why this exists

Hand-coded shotlists with world-coordinate camera positions produce
random framing. A camera primitive like `tracking_side(omniquad, side='left')`
queries the supervisor for OmniQuad's actual world pose, places the camera
off OmniQuad's heading-relative left flank at a distance scaled to OmniQuad's
body size, and moves the camera with OmniQuad's predicted motion. The
agent writes intent (`{"beat": "action"}`) and the pipeline resolves
intent into geometry.

## Agent Build Films — the locked educational story layer

When the video is the story of an AI agent building something in OmniSim, use
the Agent Build Film workflow instead of the ordinary title/end-card branding:

```bash
python -m omnisim cinema agent-build-new --title "What I built with OmniQuad" > agent_build.json
python -m omnisim cinema agent-build-validate agent_build.json
python -m omnisim cinema agent-build-make agent_build.json
```

This layer consumes real captures from `scripts/capture/` or the Cinema
director and applies the approved simulator-first three-act story, silent 0–10
second signature, purposeful direct-cut grammar, sparse information cards,
natural local voice, restrained score, claim boundaries, a clean story ending,
provenance hashes, and fail-closed delivery gate. The validator requires at
least 75 percent real simulator footage and an Act III simulator climax. The
exact workflow and editorial rules are in
[`AGENT_BUILD_FILMS.md`](AGENT_BUILD_FILMS.md).

`agent-build-make` is the benchmark path. It probes every source range and
narration budget, reuses unchanged narration blocks and edit parts, renders the
exact 720p15 proxy, checks motion/exposure/framing/cut boundaries, writes the
overview and exact cut-pair sheets, and only then unlocks the 1080p30 release
render. The ready-simulation target is 35 minutes; normal revisions target five
minutes. A cached benchmark proxy rebuild measures about two seconds on the
reference Windows workstation.

For a new shot list, keep one capture service alive and use a resumable plan:

```bash
python -m omnisim capture
python -m omnisim cinema agent-build-capture-new --world worlds/build.omniworld > capture_plan.json
python -m omnisim cinema agent-build-make agent_build.json --capture-plan capture_plan.json
```

## The native storyboard DSL

This is the explicit native camera director's format. New cinematic replay
manifests use the schema documented in [CINEMATIC_REPLAY.md](CINEMATIC_REPLAY.md).
Migrate old native storyboards by adding `"renderer": "native"` or passing
`--renderer native` to `render`; there is no automatic native fallback.

```json
{
  "renderer": "native",
  "title": "OmniQuad RL hero",
  "world": "projects/policies/research/worlds/omniquad_rl_deploy.omniworld",
  "subject": "omniquad",
  "look": "teal_orange",
  "aspect_ratios": ["16:9", "9:16", "1:1"],
  "brand": {"title_card": true, "end_slate": true, "watermark": false},
  "fps": 30,
  "world_settle_steps": 80,
  "shots": [
    {"beat": "establish", "duration_s": 6,
     "params": {"azimuth_deg": 35, "elevation_deg": 18}},
    {"beat": "action", "duration_s": 8, "params": {"side": "left"}},
    {"beat": "hero", "duration_s": 5},
    {"beat": "resolve", "duration_s": 8}
  ]
}
```

Every shot needs a `beat`. Everything else is optional — the beat picks
a default `shot` primitive and default `duration_s`, and the
storyboard-level `look` picks the default lens. Override any of them
per-shot when you want something specific.

### Catalogue

Discover what's available without leaving the CLI:

```bash
python -m omnisim cinema beats         # story beats with their default shots
python -m omnisim cinema primitives    # camera move primitives
python -m omnisim cinema looks         # named looks (lens + color grade)
python -m omnisim cinema subjects      # robot profiles + aliases
python -m omnisim cinema inspect projects/samples/demos/worlds/<world>.wbt
                                       # → JSON: every robot the world contains
```

## The pieces

| Module | Role |
|---|---|
| [`replay_record.py`](replay_record.py) | Read-only recording of named rigid-body world poses. |
| [`replay.py`](replay.py) | Replay manifests, evidence validation, proxy/final jobs, review and receipts. |
| [`blender_replay.py`](blender_replay.py) | Bind and bake measured poses, verify them, render with Cycles. |
| [`subjects.py`](subjects.py) | Robot profiles (char dim, eye height, personality) + live pose lookup via the supervisor's `/world/subject` endpoint. |
| [`lenses.py`](lenses.py) | Focal-length presets (16mm … 200mm) + FoV math. |
| [`looks.py`](looks.py) | Named looks: lens preset + ffmpeg color-grade filter chain. |
| [`composition.py`](composition.py) | Rule-of-thirds, headroom, distance-for-frame-size math. |
| [`camera.py`](camera.py) | Camera-move primitives — pure functions from subject pose → keyframes. |
| [`beats.py`](beats.py) | Story beats: a higher-level intent that picks defaults across shot + duration. |
| [`storyboard.py`](storyboard.py) | DSL parser + validation + starter-template generator. |
| [`shot.py`](shot.py) | Renders one shot to one mp4 via the capture service. |
| [`director.py`](director.py) | The orchestrator. Load world → render shots → critique → edit. |
| [`critique.py`](critique.py) | Vision-model review (Claude). Flags weak shots, suggests reshoots. |
| [`grade.py`](grade.py), [`brand.py`](brand.py), [`edit.py`](edit.py) | Post: color grade, title/end cards, multi-aspect assembly. |
| [`agent_build.py`](agent_build.py) | Locked Agent Build manifest, intro, no-outro evidence edit, mix, receipt, and release gate. |
| [`agent_build_voice.py`](agent_build_voice.py) | Pinned local natural narration with authored editorial windows. |
| [`cli.py`](cli.py) | `python -m omnisim cinema` dispatch. |

## The critique loop

If `ANTHROPIC_API_KEY` (or `OMNI_KEY`) is set and the `anthropic`
Python package is installed, the director runs each rendered shot
through Claude Haiku 4.5 with a cinematographer's-eye prompt. The
model returns 1-5 scores (subject visibility, composition, exposure,
framing intent) plus one suggested camera adjustment.

Shots that score below 3 on any axis (or below 3.5 overall) get
reshot once with the suggested adjustment. The pipeline ships the
better of the two takes.

Skip the loop with `--no-critique`.

## Operational notes

- **Native resolution uses a Camera device** at the requested size. If it is
  unavailable, the capture service discloses a viewport-bound export fallback.
  Blender replay resolution is independent of the simulator viewport.
- **One world load per storyboard.** The first shot's lens FoV sets
  the viewport FoV; subsequent shots inherit it. Mid-storyboard lens
  changes will reframe slightly. Group shots by lens family within a
  storyboard, or render hero/closeup as separate storyboards and edit
  them together.
- **Critique is optional.** No API key → loop skipped silently;
  storyboard still produces output.
- **World-load failures belong to native capture.** Check current doctor and
  run logs rather than assuming an old world-specific failure still applies.

## When to reach for this vs `scripts/capture/`

- **Capture service** — when you want a one-off still or a shot list
  with hand-coded camera coordinates. Lower-level, smaller surface area.
- **Cinema pipeline** — when you want a *video* that tells a story
  about a subject. Subject-relative, vocabulary-driven, branded,
  multi-aspect. Agent-friendly.
