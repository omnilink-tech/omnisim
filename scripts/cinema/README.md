# scripts/cinema/ — agent-driven cinematic pipeline

The capture service in [`scripts/capture/`](../capture/README.md) is the
*renderer* — it knows how to move a camera to a pose and dump frames.
This package is the *director* on top: cinematic vocabulary, subject
tracking, named looks, brand-aware assembly, multi-aspect deliverables,
and a vision-critique reshoot loop.

Entry point: **`python -m omnisim cinema <subcommand>`**.

## The 60-second tour

1. Start the capture service once and leave it running:

   ```bash
   python -m omnisim capture --port 6791 &
   ```

2. Get a starter storyboard:

   ```bash
   python -m omnisim cinema new --title "OmniQuad RL hero" --subject omniquad \
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
natural local voice, restrained score, claim boundaries, GitHub-only outro,
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

## The storyboard DSL

```json
{
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
| [`agent_build.py`](agent_build.py) | Locked Agent Build manifest, intro/outro, evidence edit, mix, receipt, and release gate. |
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

- **Resolution is viewport-bound** (typically 1896×1184). The
  `cam.enable()` segfault in the underlying Webots build forces us
  through `Supervisor.exportImage` instead of a Camera device. Output
  is letterboxed/cropped to the deliverable aspects during edit.
- **One world load per storyboard.** The first shot's lens FoV sets
  the viewport FoV; subsequent shots inherit it. Mid-storyboard lens
  changes will reframe slightly. Group shots by lens family within a
  storyboard, or render hero/closeup as separate storyboards and edit
  them together.
- **Critique is optional.** No API key → loop skipped silently;
  storyboard still produces output.
- **Worlds that segfault in the capture pipeline** (currently
  `warehouse_industrial.omniworld`) will fail cleanly with a "world load
  failed" message. Pick a different world or wait for the upstream
  Webots fix.

## When to reach for this vs `scripts/capture/`

- **Capture service** — when you want a one-off still or a shot list
  with hand-coded camera coordinates. Lower-level, smaller surface area.
- **Cinema pipeline** — when you want a *video* that tells a story
  about a subject. Subject-relative, vocabulary-driven, branded,
  multi-aspect. Agent-friendly.
