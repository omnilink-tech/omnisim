# OmniLink command demo style

Owner-approved on 2026-09-21 from the single-Husky minimal draft. This is the
default for **OmniLink command demonstration videos**, unless the owner asks
for another treatment. It overrides the general Blender beauty-render route
for this specific type of film.

## The format

**Give the prompt. Show the action.** Use one robot in one simple arena, filmed
entirely with OmniSim's native renderer. Keep a fixed, oblique camera so motion
and turns are easy to read. The approved reference is
[`styles/omnilink-minimal-reference.png`](styles/omnilink-minimal-reference.png).

- Silent: no narration, music, or sound effects by default.
- Full-frame simulation, a small OmniLink signature at upper left, and one
  rounded prompt card near the bottom. Remove the card after it is read.
- For questions or requests for clarification, show the actual robot answer
  in the same area, with a small HUSKY (or other robot name) label. A faithful
  excerpt is allowed; retain the complete original reply with the evidence.
- No telemetry panels, numbered chapters, explainer paragraphs, subtitles
  narrating the obvious, title sequence, or large closing card.
- Build interest with harder operator instructions and visible behavior:
  ordered motions, corrections, mixed units, navigation, interruptions,
  questions, and clarification. Include only behavior verified in the take.
- Keep the robot large enough to see its wheels and heading. Preserve room
  for its full route. Use the same arena and camera throughout a sequence.

## Visual recipe

Reference canvas: 1600×900, 24 fps. Final exports may scale to 1920×1080.
Segoe UI regular, or a metrically similar clean sans serif. Signature: 24 px
at (36, 28). Prompt: 38 px white text on RGBA(12, 20, 29, 235), corner radius
18 px, 32 px horizontal padding, centered around y=807. Answer: 30 px with
a 16 px muted turquoise robot label. Scale these values with the canvas.
Wrap long commands at natural clause boundaries; at most two lines. The
reference renderer is [`styles/omnilink_minimal.py`](styles/omnilink_minimal.py).

## Evidence and editing

Drive the robot through its real `/prompt` endpoint. Record prompt dispatch,
responses, timestamps, robot state and native source frames. Use those records
to align the edit. Never replace a refusal with a successful answer or a failed
movement with invented animation. Keep edited reading holds, time compression,
model/parser mode and limitations in the accompanying production notes; do not
turn the screen into an evidence dashboard. No hardware or model claims should
be implied by a parser-only simulated run.

Start with a short draft when establishing a new direction. Once the owner
approves it, finish within that direction without requesting repeat approval.
Review representative frames and decode the entire MP4; verify there is no
audio stream. Keep source material for later edits. Do not publish unless asked.

## Live model recordings

An OmniKey identifies the account; check `python -m omnisim byok` for its
connected model providers before choosing an engine. Set `OMNILINK_ENGINE`
and a matching `OMNILINK_MODEL` (an empty model lets that engine choose its
default). The bridge's implicit model is Gemini-specific. Inject the key
through the process environment; never save it in a launcher or production notes.

The deterministic parser answers confident commands by default, so a model
demonstration must opt out of it: set `OMNISIM_BRIDGE_PARSER_FIRST=0`, retain
the relay trace with `OMNILINK_TRACE`, and verify `/usage` reports zero parser
short-circuits. A successful live state tool call is the connection check;
an initialized relay alone proves nothing about provider access.

The mobile bridge accepts `POST /prompt {"text": "...", "timeout_s": 600}`
for long physical sequences during capture. The default remains 90 seconds;
the explicit budget must be positive and no greater than 600 seconds. Give
the HTTP client a slightly longer timeout. A cancelled partial sequence is a
failed take: preserve its evidence and reset the recording deliberately;
never retry a timed-out command automatically on a robot that may still move.
