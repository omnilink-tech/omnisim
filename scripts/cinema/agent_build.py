# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Manifest-driven editor for OmniSim Agent Build films.

This is the reusable form of the approved v9 milestone.  Capture is handled by
``scripts/capture`` or ``omnisim cinema render``; this module turns those real
captures into a simulator-first, three-act build story with the locked silent
intro, purposeful direct-cut grammar, sparse information cards, natural
narration mix, clean story ending, edit-decision list, provenance manifest, and
hash-bound receipt.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import shutil
import subprocess
import time
import wave
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[2]
W, H, FPS = 1920, 1080, 30
PW, PH, PFPS = 1280, 720, 15
SAFE = 72
INTRO_DURATION_S = 10.0
DISCLOSURE_DURATION_S = 5.0
DISCLOSURE = "This video was made end-to-end by an AI agent under human monitoring."
SIGNATURE_LINES = ("A real build.", "The story of an agent.", "Told by an agent.")
GITHUB_DESTINATION = "github.com/omnilink-tech/omnisim"
STYLE_VERSION = "agent_build_v9"
RENDER_CACHE_VERSION = 3
INTRO_SCORE_GAIN = 0.08
STORY_SCORE_GAIN = 0.32
INTRO_MASTER_GAIN = 0.50

INK = (242, 246, 250)
MUTED = (164, 176, 188)
CYAN = (73, 196, 220)
BLUE = (104, 154, 218)
AMBER = (224, 163, 80)
GREEN = (90, 191, 139)
RED = (220, 104, 96)
BG = (9, 13, 18)
PANEL = (18, 24, 31)
ACCENTS = {"cyan": CYAN, "blue": BLUE, "amber": AMBER,
           "green": GREEN, "red": RED}

FONT_REG = REPO_ROOT / "resources" / "branding" / "omnilink" / "fonts" / "Montserrat-Regular.ttf"
FONT_SEMI = REPO_ROOT / "resources" / "branding" / "omnilink" / "fonts" / "Montserrat-SemiBold.otf"
FONT_BOLD = REPO_ROOT / "resources" / "branding" / "omnilink" / "fonts" / "Montserrat-Bold.ttf"

REQUIRED_BEATS = ("question", "attempt", "control", "evidence", "method", "boundary", "conclusion")
ALLOWED_BEATS = set(REQUIRED_BEATS) | {"transfer"}
ALLOWED_KINDS = {"clip", "plate"}
ALLOWED_COVERAGE = {"", "wide", "medium", "detail"}


@dataclass(frozen=True)
class Overlay:
    eyebrow: str
    headline: str
    accent: str = "cyan"


@dataclass(frozen=True)
class Row:
    label: str
    detail: str
    accent: str = "cyan"


@dataclass(frozen=True)
class Segment:
    id: str
    beat: str
    kind: str
    duration_s: float
    purpose: str
    source: str | None = None
    source_in_s: float = 0.0
    source_tail_hold_s: float = 0.0
    eyebrow: str = ""
    headline: str = ""
    body: str = ""
    rows: tuple[Row, ...] = ()
    overlay: Overlay | None = None
    replay: bool = False
    replay_label: str = ""
    claim_boundary: str = ""
    act: int = 0
    coverage: str = ""
    # A lower-third caption burned into the picture for the whole segment (at most
    # CAPTION_MAX_LINES lines): how a voice="none" film tells its story. Not a subtitle stream.
    caption: str = ""


@dataclass(frozen=True)
class VoiceBlock:
    start_s: float
    window_end_s: float
    speed: float = 0.98
    sentence_pause_s: float = 0.55
    clause_pause_s: float = 0.22


@dataclass(frozen=True)
class VoiceSpec:
    script: str
    wav: str
    voice: str = "am_michael"
    blocks: tuple[VoiceBlock, ...] = ()
    # "narrated" (the default: a TTS narration mixed over a ducked score) or "none": no
    # narration at all, the story carried by on-screen captions and plates over the score.
    # A "none" manifest must say why (`reason`); the narration-only checks are then skipped
    # and every report records that they were skipped, never that they passed.
    mode: str = "narrated"
    reason: str = ""
    # Who voices a narrated film: "kokoro" (the default, generated here) or "omnivoice":
    # blocks already recorded by Omnivoice (social/Omnivoice, `omnivoice build`), whose
    # manifest `source` names; agent-build-voice only places them in their windows.
    engine: str = "kokoro"
    source: str = ""

    @property
    def narrated(self) -> bool:
        return self.mode != "none"


VOICE_ENGINES = ("kokoro", "omnivoice")


VOICE_MODES = ("narrated", "none")
# The checks that exist only because a narration exists; skipped (and reported) without one.
NARRATION_ONLY_CHECKS = (
    "voice block windows (first block at the story start, no overlap, natural speed)",
    "narration block count and per-block duration estimates (preflight)",
    "narration WAV and timing receipt present (render)",
    "narration WAV silent through the locked intro, mono (render, verify)",
    "score side-chain ducking under speech (mix)",
)
CAPTION_SIZE = 30
CAPTION_MAX_LINES = 2
CAPTION_MAX_WIDTH = 1500
CAPTION_READ_WPS = 3.0           # a calm reading pace for an on-screen caption
CAPTION_READ_LEAD_S = 0.8        # time to find the caption after a cut


def _measured_block_durations(spec: "AgentBuildSpec", count: int) -> list[float] | None:
    """The generated narration's own block lengths (its timing receipt beside the WAV), if it exists
    and matches the script block for block; else None and the preflight estimate applies."""
    try:
        path = _resolve(spec, str(Path(spec.voice.wav).with_suffix(".segments.json")))
        receipt = json.loads(path.read_text(encoding="utf-8"))
        lengths = [float(row["duration_s"]) for row in receipt.get("segments", [])]
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return lengths if len(lengths) == count else None


def _voice_record(spec: "AgentBuildSpec") -> dict[str, Any]:
    """What every report says about the narration: present, or absent and which checks were skipped."""
    if spec.voice.narrated:
        return {"mode": "narrated"}
    return {"mode": "none", "reason": spec.voice.reason, "skipped_checks": list(NARRATION_ONLY_CHECKS)}


@dataclass(frozen=True)
class AgentBuildSpec:
    source_path: Path
    title: str
    slug: str
    question: str
    claim: str
    boundary: str
    evidence: tuple[str, ...]
    segments: tuple[Segment, ...]
    voice: VoiceSpec
    repository: str = GITHUB_DESTINATION
    structure: str = ""
    simulator_footage_ratio_min: float = 0.0
    climax_segment: str = ""
    spatial_reorientation_required: bool = False
    max_consecutive_detail: int = 2
    wide_reference_segments: tuple[str, ...] = ()
    fps: int = FPS
    width: int = W
    height: int = H
    opening: str = "signature"
    # "full" (default): the action opening's disclosure line and the boundary beat's
    # band are drawn on the frame. "none": a clean frame, by the owner's choice for a
    # film (2026-11-05, the SparkRover film). Both texts stay required in the manifest
    # and are carried in verification.json, for the video's description.
    on_screen_text: str = "full"

    @property
    def intro_duration_s(self) -> float:
        return 0.0 if self.opening == "action" else INTRO_DURATION_S

    @property
    def content_duration_s(self) -> float:
        return sum(segment.duration_s for segment in self.segments)

    @property
    def duration_s(self) -> float:
        return self.intro_duration_s + self.content_duration_s

    @property
    def simulator_footage_ratio(self) -> float:
        if self.content_duration_s <= 0:
            return 0.0
        return sum(
            segment.duration_s for segment in self.segments if segment.kind == "clip"
        ) / self.content_duration_s


@dataclass(frozen=True)
class RenderProfile:
    """One deterministic output tier.

    The proxy uses the exact edit, overlays, narration, and score at half-size
    and half-rate.  It is therefore useful for editorial review while being
    materially cheaper than the release encode.
    """

    name: str
    width: int
    height: int
    fps: int
    preset: str
    crf: int


FINAL_PROFILE = RenderProfile("final", W, H, FPS, "medium", 13)
PROXY_PROFILE = RenderProfile("proxy", PW, PH, PFPS, "veryfast", 22)


def _slugify(value: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9_-]+", "_", value.strip().lower()).strip("_")
    return slug or "agent_build"


def _accent(name: str) -> tuple[int, int, int]:
    if name not in ACCENTS:
        raise ValueError(f"unknown accent {name!r}; use one of {sorted(ACCENTS)}")
    return ACCENTS[name]


def _parse_row(raw: dict[str, Any], segment_id: str) -> Row:
    label = str(raw.get("label", "")).strip()
    detail = str(raw.get("detail", "")).strip()
    if not label or not detail:
        raise ValueError(f"segment {segment_id!r} has a row without label/detail")
    accent = str(raw.get("accent", "cyan"))
    _accent(accent)
    return Row(label, detail, accent)


def _parse_segment(raw: dict[str, Any], index: int) -> Segment:
    segment_id = str(raw.get("id", "")).strip()
    beat = str(raw.get("beat", "")).strip()
    kind = str(raw.get("kind", "")).strip()
    purpose = str(raw.get("purpose", "")).strip()
    duration = float(raw.get("duration_s", 0))
    if not segment_id or not purpose:
        raise ValueError(f"segment #{index} needs a non-empty id and purpose")
    if beat not in ALLOWED_BEATS:
        raise ValueError(f"segment {segment_id!r} has unknown beat {beat!r}")
    if kind not in ALLOWED_KINDS:
        raise ValueError(f"segment {segment_id!r} has unknown kind {kind!r}")
    if duration <= 0:
        raise ValueError(f"segment {segment_id!r} needs duration_s > 0")

    source = str(raw.get("source", "")).strip() or None
    if kind == "clip" and source is None:
        raise ValueError(f"clip segment {segment_id!r} needs a source")
    source_tail_hold_s = float(raw.get("source_tail_hold_s", 0))
    if source_tail_hold_s < 0 or source_tail_hold_s > 3.0:
        raise ValueError(
            f"segment {segment_id!r} source_tail_hold_s must stay between 0 and 3 seconds"
        )
    headline = str(raw.get("headline", "")).strip()
    if kind == "plate" and not headline:
        raise ValueError(f"plate segment {segment_id!r} needs a headline")
    body = str(raw.get("body", "")).strip()
    if headline and _text_width(headline, 38, "bold") > 1160:
        raise ValueError(f"segment {segment_id!r} headline exceeds the reviewed layout width")
    if body and _text_width(body, 19) > 1160:
        raise ValueError(f"segment {segment_id!r} body exceeds the reviewed layout width")

    overlay = None
    overlay_raw = raw.get("overlay")
    if overlay_raw:
        eyebrow = str(overlay_raw.get("eyebrow", "")).strip()
        overlay_headline = str(overlay_raw.get("headline", "")).strip()
        accent = str(overlay_raw.get("accent", "cyan"))
        _accent(accent)
        if not eyebrow or not overlay_headline:
            raise ValueError(f"segment {segment_id!r} overlay needs eyebrow/headline")
        if (_text_width(eyebrow.upper(), 18, "semi") > 630 or
                _text_width(overlay_headline, 27, "bold") > 630):
            raise ValueError(f"segment {segment_id!r} overlay text exceeds its safe card")
        overlay = Overlay(eyebrow, overlay_headline, accent)

    replay = bool(raw.get("replay", False))
    replay_label = str(raw.get("replay_label", "")).strip()
    if replay and "ANALYTICAL REPLAY" not in replay_label.upper():
        raise ValueError(f"replay segment {segment_id!r} must visibly say ANALYTICAL REPLAY")

    rows = tuple(_parse_row(item, segment_id) for item in raw.get("rows", []))
    if len(rows) > 4:
        raise ValueError(f"segment {segment_id!r} has more than four rows")
    if any(_text_width(row.detail, 20, "bold") > 800 for row in rows):
        raise ValueError(f"segment {segment_id!r} row text exceeds the reviewed layout width")
    claim_boundary = str(raw.get("claim_boundary", "")).strip()
    if claim_boundary and _text_width(claim_boundary.upper(), 18, "semi") > 1680:
        raise ValueError(f"segment {segment_id!r} claim boundary exceeds the 72 px safe area")
    act = int(raw.get("act", 0))
    if act not in {0, 1, 2, 3}:
        raise ValueError(f"segment {segment_id!r} act must be 1, 2, or 3")
    coverage = str(raw.get("coverage", "")).strip().lower()
    if coverage not in ALLOWED_COVERAGE:
        raise ValueError(
            f"segment {segment_id!r} coverage must be wide, medium, or detail"
        )
    caption = " ".join(str(raw.get("caption", "")).split())
    if caption and len(_caption_lines(caption)) > CAPTION_MAX_LINES:
        raise ValueError(f"segment {segment_id!r} caption exceeds {CAPTION_MAX_LINES} lines at 1080p")
    return Segment(
        id=segment_id, beat=beat, kind=kind, duration_s=duration, purpose=purpose,
        source=source, source_in_s=float(raw.get("source_in_s", 0)),
        source_tail_hold_s=source_tail_hold_s,
        eyebrow=str(raw.get("eyebrow", "")).strip(), headline=headline,
        body=body, rows=rows, overlay=overlay,
        replay=replay, replay_label=replay_label,
        claim_boundary=claim_boundary,
        act=act,
        coverage=coverage,
        caption=caption,
    )


def parse(payload: dict[str, Any] | str | Path) -> AgentBuildSpec:
    """Parse and fail-closed validate an Agent Build Film manifest."""
    if isinstance(payload, dict):
        data = payload
        source_path = REPO_ROOT / "agent_build_manifest.json"
    else:
        source_path = Path(payload).resolve()
        data = json.loads(source_path.read_text(encoding="utf-8"))
    if int(data.get("version", 0)) != 1:
        raise ValueError("agent-build manifest version must be 1")
    title = str(data.get("title", "")).strip()
    story = data.get("story") or {}
    question = str(story.get("question", "")).strip()
    claim = str(story.get("claim", "")).strip()
    boundary = str(story.get("boundary", "")).strip()
    if not all((title, question, claim, boundary)):
        raise ValueError("title and story.question/claim/boundary are required")
    evidence = tuple(str(item).strip() for item in data.get("evidence", []) if str(item).strip())
    if not evidence:
        raise ValueError("manifest needs evidence files (result JSON, logs, world/controller, or equivalent)")
    repository = str(data.get("repository", GITHUB_DESTINATION)).strip()
    if repository != GITHUB_DESTINATION:
        raise ValueError(f"Agent Build films use the locked destination {GITHUB_DESTINATION}")
    delivery = data.get("delivery") or {}
    if (int(delivery.get("fps", FPS)), int(delivery.get("width", W)),
            int(delivery.get("height", H))) != (FPS, W, H):
        raise ValueError("locked delivery is 1920x1080 at CFR 30 fps")

    segments = tuple(_parse_segment(item, i) for i, item in enumerate(data.get("segments", []), 1))
    if not segments:
        raise ValueError("manifest has no story segments")
    ids = [segment.id for segment in segments]
    if len(ids) != len(set(ids)):
        raise ValueError("segment ids must be unique")
    if segments[0].beat != "question" or segments[0].kind != "clip":
        raise ValueError("the first post-intro segment must be real question/build footage")
    editorial = data.get("editorial") or {}
    opening = str(editorial.get("opening", "signature"))
    if opening not in ("signature", "action"):
        raise ValueError("editorial.opening must be 'signature' or 'action'")
    on_screen_text = str(editorial.get("on_screen_text", "full"))
    if on_screen_text not in ("full", "none"):
        raise ValueError("editorial.on_screen_text must be 'full' or 'none'")
    intro_duration = 0.0 if opening == "action" else INTRO_DURATION_S
    # In an action-led story, show the repair before revealing its result.
    required_beats = (("question", "attempt", "control", "method", "evidence", "boundary", "conclusion")
                      if opening == "action" else REQUIRED_BEATS)
    first_beat = {beat: next((i for i, s in enumerate(segments) if s.beat == beat), None)
                  for beat in required_beats}
    missing = [beat for beat, index in first_beat.items() if index is None]
    if missing:
        raise ValueError(f"story is missing required beats: {', '.join(missing)}")
    # Action openings disclose scope when it first becomes relevant, instead
    # of requiring a disclaimer scene after the successful climax.
    ordered_beats = tuple(b for b in required_beats if b != "boundary") if opening == "action" else required_beats
    order = [int(first_beat[beat]) for beat in ordered_beats]
    if order != sorted(order):
        raise ValueError(f"required beats must first appear in order: {' -> '.join(required_beats)}")
    for beat in ("attempt", "control"):
        segment = segments[int(first_beat[beat])]
        if segment.kind != "clip":
            raise ValueError(f"the {beat} beat must show real OmniSim footage")
    boundary_segment = segments[int(first_beat["boundary"])]
    if not boundary_segment.claim_boundary:
        raise ValueError("the boundary beat must carry an exact on-screen claim_boundary")

    editorial = data.get("editorial") or {}
    structure = str(editorial.get("structure", "")).strip()
    simulator_ratio_min = float(editorial.get("simulator_footage_ratio_min", 0.0))
    climax_segment = str(editorial.get("climax_segment", "")).strip()
    spatial_reorientation_required = bool(
        editorial.get("spatial_reorientation_required", False)
    )
    max_consecutive_detail = int(editorial.get("max_consecutive_detail", 2))
    wide_reference_segments = tuple(
        str(item).strip()
        for item in editorial.get("wide_reference_segments", [])
        if str(item).strip()
    )
    if structure:
        if structure != "three_act":
            raise ValueError("editorial.structure must be 'three_act'")
        acts = [segment.act for segment in segments]
        if any(act == 0 for act in acts):
            raise ValueError("three-act films must assign act 1, 2, or 3 to every segment")
        if acts != sorted(acts) or set(acts) != {1, 2, 3}:
            raise ValueError("three-act segment acts must be contiguous and ordered 1 -> 2 -> 3")
        if not 0.5 <= simulator_ratio_min <= 1.0:
            raise ValueError("simulator_footage_ratio_min must stay between 0.5 and 1.0")
        clip_seconds = sum(segment.duration_s for segment in segments if segment.kind == "clip")
        footage_ratio = clip_seconds / sum(segment.duration_s for segment in segments)
        if footage_ratio + 1e-9 < simulator_ratio_min:
            raise ValueError(
                f"simulator footage ratio {footage_ratio:.3f} is below the required "
                f"{simulator_ratio_min:.3f}"
            )
        climax = next((segment for segment in segments if segment.id == climax_segment), None)
        if climax is None or climax.kind != "clip" or climax.act != 3:
            raise ValueError("three-act films need an Act III clip named by editorial.climax_segment")
        if climax.duration_s < 12.0:
            raise ValueError("the Act III climax clip must be at least 12 seconds")

        if spatial_reorientation_required:
            clips = [segment for segment in segments if segment.kind == "clip"]
            missing_coverage = [segment.id for segment in clips if not segment.coverage]
            if missing_coverage:
                raise ValueError(
                    "spatial reorientation requires coverage on every clip: "
                    + ", ".join(missing_coverage)
                )
            if max_consecutive_detail < 1:
                raise ValueError("max_consecutive_detail must be at least 1")
            run = 0
            for segment in clips:
                run = run + 1 if segment.coverage == "detail" else 0
                if run > max_consecutive_detail:
                    raise ValueError(
                        "too many consecutive detail clips without a wide or medium reorientation"
                    )
            if not wide_reference_segments:
                raise ValueError("spatial reorientation needs declared wide_reference_segments")
            wide_refs = []
            for segment_id in wide_reference_segments:
                reference = next((segment for segment in segments if segment.id == segment_id), None)
                if reference is None or reference.kind != "clip" or reference.coverage != "wide":
                    raise ValueError(
                        f"wide reference {segment_id!r} must name a wide simulator clip"
                    )
                wide_refs.append(reference)
            if not any(segment.act == 1 for segment in wide_refs):
                raise ValueError("wide references must orient Act I")
            if not any(segment.act == 3 for segment in wide_refs):
                raise ValueError("wide references must reorient Act III")

    # Source time moves forward. Reuse is only legal when declared analytical replay.
    previous_by_source: dict[str, float] = {}
    for segment in segments:
        if segment.kind != "clip" or segment.source is None:
            continue
        previous = previous_by_source.get(segment.source, -1.0)
        if segment.source_in_s < previous - 0.001 and not segment.replay:
            raise ValueError(f"segment {segment.id!r} rewinds source time without analytical replay")
        if not segment.replay:
            previous_by_source[segment.source] = segment.source_in_s + segment.duration_s

    voice_raw = data.get("voice") or {}
    voice_mode = str(voice_raw.get("mode", "narrated")).strip()
    if voice_mode not in VOICE_MODES:
        raise ValueError(f"voice.mode must be one of {', '.join(VOICE_MODES)}")
    if voice_mode == "none":
        reason = str(voice_raw.get("reason", "")).strip()
        if not reason:
            raise ValueError("voice.mode 'none' needs a voice.reason (who decided there is no narration, and why)")
        if voice_raw.get("blocks"):
            raise ValueError("voice.mode 'none' declares no voice blocks")
        if not any(segment.caption for segment in segments):
            raise ValueError("a film without narration needs on-screen captions (segment.caption) to tell its story")
        return AgentBuildSpec(
            source_path=source_path, title=title,
            slug=str(data.get("slug") or _slugify(title)), question=question,
            claim=claim, boundary=boundary, evidence=evidence, segments=segments,
            voice=VoiceSpec(script="", wav="", voice="", blocks=(), mode="none", reason=reason),
            repository=repository, opening=opening, structure=structure,
            simulator_footage_ratio_min=simulator_ratio_min, climax_segment=climax_segment,
            spatial_reorientation_required=spatial_reorientation_required,
            max_consecutive_detail=max_consecutive_detail,
            wide_reference_segments=wide_reference_segments,
        )
    script = str(voice_raw.get("script", "")).strip()
    wav = str(voice_raw.get("wav", "")).strip()
    if not script or not wav:
        raise ValueError("voice.script and voice.wav are required")
    voice_engine = str(voice_raw.get("engine", "kokoro")).strip()
    if voice_engine not in VOICE_ENGINES:
        raise ValueError(f"voice.engine must be one of {', '.join(VOICE_ENGINES)}")
    voice_source = str(voice_raw.get("source", "")).strip()
    if voice_engine == "omnivoice" and not voice_source:
        raise ValueError("voice.engine 'omnivoice' needs voice.source: the Omnivoice manifest.json")
    blocks = tuple(VoiceBlock(
        start_s=float(item["start_s"]), window_end_s=float(item["window_end_s"]),
        speed=float(item.get("speed", 0.98)),
        sentence_pause_s=float(item.get("sentence_pause_s", 0.55)),
        clause_pause_s=float(item.get("clause_pause_s", 0.22)),
    ) for item in voice_raw.get("blocks", []))
    if not blocks or blocks[0].start_s != intro_duration:
        raise ValueError(f"the first voice block must begin exactly at {intro_duration:.1f} seconds")
    if any(block.window_end_s <= block.start_s for block in blocks):
        raise ValueError("every voice block needs a positive editorial window")
    if any(not 0.90 <= block.speed <= 1.05 for block in blocks):
        raise ValueError("natural narration speed must stay between 0.90 and 1.05")
    for left, right in zip(blocks, blocks[1:]):
        if left.window_end_s > right.start_s:
            raise ValueError("voice editorial windows overlap")
    content_end = intro_duration + sum(segment.duration_s for segment in segments)
    if any(block.start_s < intro_duration or block.window_end_s > content_end for block in blocks):
        raise ValueError("voice blocks must stay within the narrated story content")

    return AgentBuildSpec(
        source_path=source_path, title=title,
        slug=str(data.get("slug") or _slugify(title)), question=question,
        claim=claim, boundary=boundary, evidence=evidence, segments=segments,
        voice=VoiceSpec(script=script, wav=wav,
                        voice=str(voice_raw.get("voice", "am_michael")), blocks=blocks,
                        engine=voice_engine, source=voice_source),
        repository=repository,
        opening=opening,
        on_screen_text=on_screen_text,
        structure=structure,
        simulator_footage_ratio_min=simulator_ratio_min,
        climax_segment=climax_segment,
        spatial_reorientation_required=spatial_reorientation_required,
        max_consecutive_detail=max_consecutive_detail,
        wide_reference_segments=wide_reference_segments,
    )


def template(title: str = "My OmniSim Agent Build") -> dict[str, Any]:
    """Return a complete starter manifest with placeholders for real captures."""
    return {
        "version": 1,
        "title": title,
        "slug": _slugify(title),
        "repository": GITHUB_DESTINATION,
        "story": {
            "duration_policy": "story_driven",
            "question": "What measurable robotics question does this build answer?",
            "claim": "What does the controlled comparison support?",
            "boundary": "What does this simulation not establish?",
        },
        "editorial": {
            "structure": "three_act",
            "simulator_footage_ratio_min": 0.75,
            "climax_segment": "climax",
            "spatial_reorientation_required": True,
            "max_consecutive_detail": 2,
            "wide_reference_segments": ["build_question", "climax"],
        },
        "evidence": [
            "evidence/result.json",
            "evidence/controller.log",
            "worlds/build.omniworld"
        ],
        "delivery": {"width": W, "height": H, "fps": FPS},
        "voice": {
            "script": "narration.txt", "wav": "build/narration.wav",
            "voice": "am_michael",
            "blocks": [
                {"start_s": 10.0, "window_end_s": 20.0, "speed": 0.98},
                {"start_s": 22.0, "window_end_s": 32.0, "speed": 0.97},
                {"start_s": 34.0, "window_end_s": 44.0, "speed": 0.97},
                {"start_s": 46.0, "window_end_s": 56.0, "speed": 0.97},
                {"start_s": 58.0, "window_end_s": 68.0, "speed": 0.97},
                {"start_s": 70.0, "window_end_s": 78.0, "speed": 0.97},
                {"start_s": 80.0, "window_end_s": 87.5, "speed": 0.97},
            ],
        },
        "segments": [
            {"id": "build_question", "beat": "question", "kind": "clip",
             "act": 1, "coverage": "wide",
             "duration_s": 12.0, "source": "captures/question.mp4", "source_in_s": 0,
             "purpose": "Begin on the real object or robot and make the measurable question judgeable.",
             "overlay": {"eyebrow": "MY BUILD LOG · QUESTION",
                         "headline": "ONE BUILD. ONE TEST.", "accent": "cyan"}},
            {"id": "first_attempt", "beat": "attempt", "kind": "clip",
             "act": 1, "coverage": "detail",
             "duration_s": 12.0, "source": "captures/attempt.mp4", "source_in_s": 0,
             "purpose": "Show the apparent success in a locked, judgeable frame."},
            {"id": "controlled_failure", "beat": "control", "kind": "clip",
             "act": 2, "coverage": "wide",
             "duration_s": 12.0, "source": "captures/control.mp4", "source_in_s": 0,
             "purpose": "Change one variable and show the controlled failure.",
             "overlay": {"eyebrow": "NEGATIVE CONTROL",
                         "headline": "ONE VARIABLE CHANGED", "accent": "amber"}},
            {"id": "evidence", "beat": "evidence", "kind": "plate",
             "act": 2, "duration_s": 6.0, "eyebrow": "WHAT THE COMPARISON EARNED",
             "headline": "THE RESULT BECAME EVIDENCE",
             "body": "Success and control are readable only together.",
             "purpose": "State the bounded claim supported by the comparison.",
             "rows": [
                 {"label": "SUCCESS", "detail": "Describe the observed result", "accent": "green"},
                 {"label": "CONTROL", "detail": "Describe the opposite outcome", "accent": "amber"},
                 {"label": "BOUNDARY", "detail": "State what remains unproven", "accent": "red"}
             ]},
            {"id": "agent_method", "beat": "method", "kind": "clip",
             "act": 2, "coverage": "detail", "duration_s": 12.0, "source": "captures/method.mp4",
             "source_in_s": 0,
             "purpose": "Narrate the build method over purposeful simulator footage."},
            {"id": "climax", "beat": "evidence", "kind": "clip",
             "act": 3, "coverage": "wide", "duration_s": 32.0, "source": "captures/climax.mp4",
             "source_in_s": 0,
             "purpose": "Stay with the robot through the decisive continuous run to the result."},
            {"id": "claim_boundary", "beat": "boundary", "kind": "clip",
             "act": 3, "coverage": "wide", "duration_s": 10.0, "source": "captures/boundary.mp4",
             "source_in_s": 0,
             "purpose": "Make claim limits part of the story.",
             "claim_boundary": "Replace with the exact build-specific boundary."},
            {"id": "conclusion", "beat": "conclusion", "kind": "clip",
             "act": 3, "coverage": "detail", "duration_s": 10.0, "source": "captures/conclusion.mp4",
             "source_in_s": 0,
             "purpose": "Conclude with the reusable build method, not a capability montage."},
        ],
    }


def _resolve(spec: AgentBuildSpec, value: str) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    local = (spec.source_path.parent / path).resolve()
    if local.exists():
        return local
    return (REPO_ROOT / path).resolve()


def _ffmpeg() -> str:
    binary = shutil.which("ffmpeg")
    if not binary:
        raise RuntimeError("ffmpeg is required on PATH")
    return binary


def _ffprobe() -> str:
    binary = shutil.which("ffprobe")
    if not binary:
        raise RuntimeError("ffprobe is required on PATH")
    return binary


def _run(args: list[str]) -> None:
    process = subprocess.run(args, cwd=REPO_ROOT, text=True, capture_output=True)
    if process.returncode:
        raise RuntimeError(f"command failed ({process.returncode}): {' '.join(args[:12])}\n{process.stderr[-3000:]}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stable_hash(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class ArtifactCache:
    """Small content-addressed cache local to one rendered deliverable.

    Generated media remains beside the local build and out of Git.  A cache
    hit requires both an identical semantic key and a non-empty artifact, so
    interrupted encodes never masquerade as reusable work.
    """

    def __init__(self, root: Path, profile: RenderProfile):
        self.path = root / ".agent_build_cache.json"
        self.profile = profile
        try:
            self.data = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            self.data = {"version": RENDER_CACHE_VERSION, "artifacts": {}}
        if int(self.data.get("version", 0)) != RENDER_CACHE_VERSION:
            self.data = {"version": RENDER_CACHE_VERSION, "artifacts": {}}
        self.hits = 0
        self.misses = 0
        self.events: list[dict[str, Any]] = []

    def key(self, kind: str, payload: Any) -> str:
        return _stable_hash({
            "cache_version": RENDER_CACHE_VERSION,
            "style": STYLE_VERSION,
            "profile": self.profile.__dict__,
            "kind": kind,
            "payload": payload,
        })

    def hit(self, label: str, key: str, output: Path) -> bool:
        record = self.data.get("artifacts", {}).get(label) or {}
        ok = (record.get("key") == key and output.exists()
              and output.stat().st_size > 0)
        if ok:
            actual_sha = _sha256(output)
            expected_sha = record.get("sha256")
            ok = expected_sha in (None, actual_sha)
            if expected_sha is None:
                # One-time upgrade for caches written before output hashes
                # became part of the reuse contract.
                record["sha256"] = actual_sha
                self.path.write_text(
                    json.dumps(self.data, indent=2) + "\n", encoding="utf-8"
                )
        if ok:
            self.hits += 1
        else:
            self.misses += 1
        self.events.append({"artifact": label, "cache_hit": ok,
                            "path": str(output)})
        return ok

    def store(self, label: str, key: str, output: Path) -> None:
        self.data.setdefault("artifacts", {})[label] = {
            "key": key, "path": str(output), "bytes": output.stat().st_size,
            "sha256": _sha256(output),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.data, indent=2) + "\n", encoding="utf-8")

    def report(self) -> dict[str, Any]:
        return {"hits": self.hits, "misses": self.misses, "events": self.events}


def _rate(value: str | None) -> float:
    if not value or value == "0/0":
        return 0.0
    numerator, separator, denominator = value.partition("/")
    if not separator:
        return float(value)
    return float(numerator) / float(denominator)


def probe_media(path: Path) -> dict[str, Any]:
    """Return the media facts needed by the fail-fast source preflight."""
    process = subprocess.run([
        _ffprobe(), "-v", "error", "-show_entries",
        "format=duration:stream=codec_type,width,height,r_frame_rate,avg_frame_rate,nb_frames",
        "-of", "json", str(path),
    ], cwd=REPO_ROOT, text=True, capture_output=True)
    if process.returncode:
        raise ValueError(f"ffprobe rejected {path}: {process.stderr.strip()}")
    payload = json.loads(process.stdout)
    video = next((item for item in payload.get("streams", [])
                  if item.get("codec_type") == "video"), None)
    if video is None:
        raise ValueError(f"capture has no video stream: {path}")
    duration = float(payload.get("format", {}).get("duration") or 0.0)
    return {
        "path": str(path), "sha256": _sha256(path), "duration_s": duration,
        "width": int(video.get("width") or 0), "height": int(video.get("height") or 0),
        "fps": _rate(video.get("avg_frame_rate") or video.get("r_frame_rate")),
        "frames": int(video.get("nb_frames") or 0),
    }


def _voice_estimate(text: str, block: VoiceBlock) -> tuple[float, int]:
    """Conservative local estimate used before the TTS model is loaded."""
    words = len(re.findall(r"\b[\w'-]+\b", text))
    sentences = len(re.findall(r"[.!?]+(?:[\"']|\s|$)", text))
    clauses = len(re.findall(r"[,;:]", text))
    seconds = words / max(0.1, 2.70 * block.speed)
    seconds += max(0, sentences - 1) * block.sentence_pause_s
    seconds += clauses * block.clause_pause_s
    return seconds, words


def preflight(spec: AgentBuildSpec, *, require_captures: bool = True) -> dict[str, Any]:
    """Validate every expensive input before voice generation or rendering.

    Structural validation happens in :func:`parse`.  This second gate checks
    the filesystem and opens each unique capture once, so an impossible trim or
    an overstuffed narration block fails before any encode is started.
    """
    started = time.perf_counter()
    errors: list[str] = []
    warnings: list[str] = []
    media: dict[str, dict[str, Any]] = {}
    ranges: list[dict[str, Any]] = []
    for segment in spec.segments:
        if segment.source is None:
            continue
        source = _resolve(spec, segment.source)
        if not source.exists():
            if require_captures:
                errors.append(f"{segment.id}: capture is missing: {source}")
            continue
        key = str(source)
        if key not in media:
            try:
                media[key] = probe_media(source)
            except (ValueError, OSError, json.JSONDecodeError) as exc:
                errors.append(f"{segment.id}: {exc}")
                continue
        facts = media[key]
        requested_end = segment.source_in_s + segment.duration_s
        frame_slack = 1.0 / max(1.0, float(facts["fps"]) or FPS)
        if segment.source_in_s < 0:
            errors.append(f"{segment.id}: source_in_s cannot be negative")
        available_with_hold = float(facts["duration_s"]) + segment.source_tail_hold_s
        if requested_end > available_with_hold + frame_slack:
            errors.append(
                f"{segment.id}: requests {segment.source_in_s:.3f}-{requested_end:.3f}s "
                f"from a {facts['duration_s']:.3f}s capture with only "
                f"{segment.source_tail_hold_s:.3f}s of declared tail hold"
            )
        ranges.append({
            "segment": segment.id, "source": key,
            "source_in_s": segment.source_in_s, "source_out_s": requested_end,
            "available_s": facts["duration_s"],
        })

    evidence: list[dict[str, Any]] = []
    for value in spec.evidence:
        path = _resolve(spec, value)
        if not path.exists():
            errors.append(f"evidence is missing: {path}")
        elif path.is_file():
            evidence.append({"path": str(path), "sha256": _sha256(path)})
        else:
            errors.append(f"evidence must be a file: {path}")

    voice_blocks: list[dict[str, Any]] = []
    captions: list[dict[str, Any]] = []
    if spec.voice.narrated:
        script_path = _resolve(spec, spec.voice.script)
        if not script_path.exists():
            errors.append(f"narration script is missing: {script_path}")
        else:
            blocks = [part.strip() for part in
                      script_path.read_text(encoding="utf-8").split("\n\n") if part.strip()]
            if len(blocks) != len(spec.voice.blocks):
                errors.append(
                    f"narration has {len(blocks)} blocks; manifest declares "
                    f"{len(spec.voice.blocks)} windows"
                )
            else:
                measured = _measured_block_durations(spec, len(blocks))
                for index, (text, block) in enumerate(zip(blocks, spec.voice.blocks), 1):
                    estimate, words = _voice_estimate(text, block)
                    window = block.window_end_s - block.start_s
                    if measured is not None:
                        # The voice already exists: its measured length, not the estimate, must fit.
                        length = measured[index - 1]
                        if length > window + 0.02:
                            errors.append(f"voice block {index}: the generated {length:.2f}s exceeds its "
                                          f"{window:.2f}s window")
                        voice_blocks.append({"index": index, "words": words, "measured_duration_s": length,
                                             "estimated_duration_s": round(estimate, 3),
                                             "window_s": round(window, 3)})
                        continue
                    # The model remains the final authority.  This catches copy
                    # that is plainly too long while leaving a small estimator
                    # margin for natural pronunciation differences.
                    if estimate > window * 1.10:
                        errors.append(
                            f"voice block {index}: estimated {estimate:.2f}s exceeds its "
                            f"{window:.2f}s window; shorten the {words}-word block"
                        )
                    elif estimate > window * 0.90:
                        warnings.append(
                            f"voice block {index} is close to its limit "
                            f"({estimate:.2f}s estimated in {window:.2f}s)"
                        )
                    voice_blocks.append({
                        "index": index, "words": words,
                        "estimated_duration_s": round(estimate, 3),
                        "window_s": round(window, 3),
                    })
    else:
        # No narration: the captions carry the story, so each must be readable in its segment.
        for segment in spec.segments:
            if not segment.caption:
                continue
            words = len(re.findall(r"[\w'.-]+", segment.caption))
            read_s = words / CAPTION_READ_WPS + CAPTION_READ_LEAD_S
            if read_s > segment.duration_s:
                errors.append(f"{segment.id}: caption needs about {read_s:.1f}s to read in a "
                              f"{segment.duration_s:.1f}s segment; shorten its {words} words")
            elif read_s > segment.duration_s * 0.9:
                warnings.append(f"{segment.id}: caption is close to its reading limit "
                                f"({read_s:.1f}s in {segment.duration_s:.1f}s)")
            captions.append({"segment": segment.id, "words": words, "read_s": round(read_s, 2),
                             "lines": len(_caption_lines(segment.caption))})

    report = {
        "valid": not errors, "style": STYLE_VERSION,
        "elapsed_s": round(time.perf_counter() - started, 3),
        "errors": errors, "warnings": warnings,
        "sources": list(media.values()), "ranges": ranges,
        "evidence": evidence, "voice_blocks": voice_blocks,
        "voice": _voice_record(spec), "captions": captions,
    }
    if errors:
        raise ValueError("Agent Build preflight failed:\n- " + "\n- ".join(errors))
    return report


def _font(size: int, weight: str = "regular") -> ImageFont.FreeTypeFont:
    path = {"regular": FONT_REG, "semi": FONT_SEMI, "bold": FONT_BOLD}[weight]
    return ImageFont.truetype(str(path), size=size)


def _text_width(text: str, size: int, weight: str = "regular") -> int:
    left, _top, right, _bottom = _font(size, weight).getbbox(text)
    return right - left


def _caption_lines(text: str) -> list[str]:
    """Greedy word wrap of a caption to CAPTION_MAX_WIDTH px at the 1080p caption size."""
    lines: list[str] = []
    for word in text.split():
        trial = f"{lines[-1]} {word}" if lines else word
        if lines and _text_width(trial, CAPTION_SIZE, "semi") <= CAPTION_MAX_WIDTH:
            lines[-1] = trial
        else:
            lines.append(word)
    return lines


def _ease(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _centered(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str,
              face: ImageFont.FreeTypeFont, fill: tuple[int, ...]) -> None:
    draw.text(xy, text, font=face, fill=fill, anchor="mm")


def _plate_base() -> Image.Image:
    image = Image.new("RGB", (PW, PH), BG)
    glow = Image.new("RGBA", (PW, PH), (0, 0, 0, 0))
    gd = ImageDraw.Draw(glow)
    gd.ellipse((250, 10, 1030, 790), fill=(*CYAN, 12))
    image = Image.alpha_composite(image.convert("RGBA"), glow.filter(ImageFilter.GaussianBlur(110))).convert("RGB")
    draw = ImageDraw.Draw(image)
    for x in range(0, PW, 64):
        draw.line((x, 0, x, PH), fill=(15, 21, 27), width=1)
    for y in range(0, PH, 64):
        draw.line((0, y, PW, y), fill=(15, 21, 27), width=1)
    return image


def _render_raw_frames(output: Path, duration_s: float, painter: Any,
                       profile: RenderProfile) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{PW}x{PH}",
        "-r", str(PFPS), "-i", "-", "-vf",
        f"scale={profile.width}:{profile.height}:flags=lanczos",
        "-an", "-r", str(profile.fps),
        "-frames:v", str(round(duration_s * profile.fps)),
        "-c:v", "libx264", "-preset", profile.preset,
        "-crf", str(profile.crf), "-profile:v", "high",
        "-pix_fmt", "yuv420p", str(output),
    ]
    process = subprocess.Popen(command, cwd=REPO_ROOT, stdin=subprocess.PIPE)
    assert process.stdin is not None
    total = int(round(duration_s * PFPS))
    for frame_index in range(total):
        progress = frame_index / max(1, total - 1)
        image = painter(progress).convert("RGB")
        process.stdin.write(np.asarray(image, dtype=np.uint8).tobytes())
    process.stdin.close()
    if process.wait() != 0:
        raise RuntimeError(f"motion-graphic encode failed: {output}")
    return output


def render_locked_intro(out_dir: Path, profile: RenderProfile,
                        cache: ArtifactCache) -> tuple[Path, Path]:
    """Render the immutable 0–5 disclosure and 5–10 story signature."""
    def disclosure(progress: float) -> Image.Image:
        image = _plate_base()
        draw = ImageDraw.Draw(image)
        intro = _ease(progress / 0.18)
        line = _ease((progress - 0.10) / 0.28)
        monitor = _ease((progress - 0.30) / 0.24)
        _centered(draw, (PW // 2, 156), "AI-AGENT PRODUCTION DISCLOSURE", _font(16, "semi"), CYAN)
        width = int(260 * line)
        draw.line((PW // 2 - width, 194, PW // 2 + width, 194), fill=CYAN, width=2)
        _centered(draw, (PW // 2, 292), "THIS VIDEO WAS MADE END-TO-END", _font(37, "bold"),
                  tuple(int(value * intro) for value in INK))
        _centered(draw, (PW // 2, 352), "BY AN AI AGENT", _font(52, "bold"),
                  tuple(int(value * line) for value in INK))
        pill = int(560 * monitor)
        if pill > 4:
            draw.rounded_rectangle((PW // 2 - pill // 2, 430, PW // 2 + pill // 2, 504),
                                   radius=18, fill=PANEL, outline=GREEN, width=2)
        _centered(draw, (PW // 2, 467), "UNDER HUMAN MONITORING", _font(22, "semi"),
                  tuple(int(value * monitor) for value in GREEN))
        return image

    def signature(progress: float) -> Image.Image:
        image = Image.new("RGB", (PW, PH), (0, 0, 0))
        draw = ImageDraw.Draw(image)
        amounts = (_ease(progress / 0.18), _ease((progress - 0.16) / 0.20),
                   _ease((progress - 0.32) / 0.20))
        _centered(draw, (PW // 2, 274), SIGNATURE_LINES[0].upper(), _font(52, "bold"),
                  tuple(int(value * amounts[0]) for value in INK))
        _centered(draw, (PW // 2, 376), SIGNATURE_LINES[1].upper(), _font(30, "semi"),
                  tuple(int(value * amounts[1]) for value in INK))
        _centered(draw, (PW // 2, 444), SIGNATURE_LINES[2].upper(), _font(30, "semi"),
                  tuple(int(value * amounts[2]) for value in INK))
        return image

    outputs = (out_dir / "locked_disclosure.mp4",
               out_dir / "locked_story_signature.mp4")
    painters = (disclosure, signature)
    payloads = (
        {"text": DISCLOSURE, "duration_s": DISCLOSURE_DURATION_S},
        {"lines": SIGNATURE_LINES, "duration_s": 5.0},
    )
    for label, output, painter, payload in zip(
            ("locked_disclosure", "locked_story_signature"), outputs, painters, payloads):
        key = cache.key("locked_intro", payload)
        if not cache.hit(label, key, output):
            _render_raw_frames(output, 5.0, painter, profile)
            cache.store(label, key, output)
    return outputs


def _make_overlay(segment: Segment, out_path: Path,
                  profile: RenderProfile, *, disclosure: bool = False,
                  omnisim_brand: bool = False, boundary_band: bool = True) -> Path | None:
    claim_boundary = segment.claim_boundary if boundary_band else ""
    if (segment.overlay is None and not segment.replay_label and not claim_boundary
            and not disclosure and not segment.caption):
        return None
    image = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    ink = (246, 244, 239) if omnisim_brand else INK
    panel = (10, 10, 6) if omnisim_brand else (10, 15, 21)
    soft = (250, 246, 164)
    def face(size: int, weight: str = "regular") -> ImageFont.FreeTypeFont:
        brand_font = REPO_ROOT / "resources/branding/omnisim/fonts/SpaceGrotesk.ttf"
        if omnisim_brand and brand_font.exists():
            result = ImageFont.truetype(str(brand_font), size)
            result.set_variation_by_axes([{"regular": 400, "semi": 600, "bold": 700}[weight]])
            return result
        return _font(size, weight)
    if disclosure:
        draw.rounded_rectangle((SAFE, H - SAFE - 48, 1070, H - SAFE), radius=12,
                               fill=(*panel, 220))
        draw.text((SAFE + 20, H - SAFE - 37),
                  "AI-produced under human monitoring  /  OmniSim simulation",
                  font=face(24, "semi"), fill=ink)
    if segment.overlay:
        accent = soft if omnisim_brand else _accent(segment.overlay.accent)
        width, height = 720, 118
        x, y = W - SAFE - width, SAFE
        draw.rounded_rectangle((x + 8, y + 10, x + width, y + height), radius=17, fill=(0, 0, 0, 115))
        draw.rounded_rectangle((x, y, x + width - 14, y + 102), radius=16,
                               fill=(*panel, 239), outline=(*ink, 30), width=2)
        draw.rounded_rectangle((x, y + 18, x + 5, y + 84), radius=3, fill=(*accent, 245))
        draw.text((x + 29, y + 18), segment.overlay.eyebrow.upper(), font=face(18, "semi"), fill=accent)
        draw.text((x + 29, y + 50), segment.overlay.headline, font=face(27, "bold"), fill=ink)
    if segment.replay_label:
        draw.text((W // 2, SAFE), segment.replay_label.upper(), font=face(17, "semi"), fill=ink, anchor="ma")
    if segment.caption:
        # Lower third, centred, inside the safe area and above any claim-boundary band.
        lines = _caption_lines(segment.caption)
        cap_font = face(CAPTION_SIZE, "semi")
        line_h = round(CAPTION_SIZE * 1.42)
        box_w = max(_text_width(line, CAPTION_SIZE, "semi") for line in lines) + 64
        box_h = line_h * len(lines) + 30
        bottom = H - SAFE - (72 if claim_boundary else 0)
        x0 = (W - box_w) // 2
        draw.rounded_rectangle((x0, bottom - box_h, x0 + box_w, bottom), radius=14,
                               fill=(*panel, 214))
        for k, line in enumerate(lines):
            draw.text((W // 2, bottom - box_h + 15 + k * line_h), line, font=cap_font, fill=ink, anchor="ma")
    if claim_boundary:
        draw.rounded_rectangle((SAFE, H - SAFE - 58, W - SAFE, H - SAFE), radius=13,
                               fill=(*panel, 232), outline=(66, 65, 51) if omnisim_brand else RED, width=2)
        draw.text((W // 2, H - SAFE - 29), claim_boundary.upper(),
                  font=face(18, "semi"), fill=ink, anchor="mm")
    if (profile.width, profile.height) != (W, H):
        image = image.resize((profile.width, profile.height), Image.Resampling.LANCZOS)
    image.save(out_path)
    return out_path


def _render_clip(spec: AgentBuildSpec, segment: Segment, index: int, out_dir: Path,
                 profile: RenderProfile, cache: ArtifactCache) -> Path:
    assert segment.source is not None
    source = _resolve(spec, segment.source)
    if not source.exists():
        raise FileNotFoundError(f"capture is missing for segment {segment.id}: {source}")
    output = out_dir / f"{index:03d}_{segment.id}.mp4"
    brand_font = REPO_ROOT / "resources/branding/omnisim/fonts/SpaceGrotesk.ttf"
    key = cache.key("clip", {"source_sha256": _sha256(source),
                             "segment": asdict(segment), "opening": spec.opening,
                             "on_screen_text": spec.on_screen_text,
                             "action_brand_version": 2 if spec.opening == "action" else None,
                             "action_font_sha256": _sha256(brand_font)
                             if spec.opening == "action" and brand_font.exists() else None})
    label = f"segment:{index:03d}:{segment.id}"
    if cache.hit(label, key, output):
        return output
    overlay = _make_overlay(
        segment, out_dir / f"{index:03d}_{segment.id}_overlay.png", profile,
        disclosure=(spec.opening == "action" and index == 1 and spec.on_screen_text == "full"),
        omnisim_brand=spec.opening == "action",
        boundary_band=spec.on_screen_text == "full",
    )
    tail_filter = (f"tpad=stop_mode=clone:stop_duration={segment.source_tail_hold_s:.3f},"
                   if segment.source_tail_hold_s > 0 else "")
    base_filter = tail_filter + (
        f"scale={profile.width}:{profile.height}:force_original_aspect_ratio=increase:flags=lanczos,"
        f"crop={profile.width}:{profile.height},"
        # Action edits carry a finished photographic grade and exact brand
        # colors in their source graphics. Do not cool or desaturate them twice.
        + ("setsar=1,format=yuv420p" if spec.opening == "action" else
           "eq=contrast=1.025:saturation=0.94:gamma=0.99,format=yuv420p")
    )
    command = [
        _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-ss", f"{segment.source_in_s:.3f}", "-t", f"{segment.duration_s:.3f}",
        "-i", str(source),
    ]
    if overlay:
        command += ["-loop", "1", "-i", str(overlay), "-filter_complex",
                    f"[0:v]{base_filter}[base];[base][1:v]overlay=0:0:format=auto[outv]",
                    "-map", "[outv]"]
    else:
        command += ["-vf", base_filter]
    command += [
        "-an", "-r", str(profile.fps),
        "-frames:v", str(round(segment.duration_s * profile.fps)),
        "-c:v", "libx264", "-preset", profile.preset,
        "-crf", str(profile.crf), "-profile:v", "high",
        "-pix_fmt", "yuv420p", str(output),
    ]
    _run(command)
    cache.store(label, key, output)
    return output


def _render_plate(segment: Segment, index: int, out_dir: Path,
                  profile: RenderProfile, cache: ArtifactCache) -> Path:
    def painter(progress: float) -> Image.Image:
        image = _plate_base()
        draw = ImageDraw.Draw(image)
        draw.text((58, 70), (segment.eyebrow or segment.beat).upper(), font=_font(16, "semi"), fill=CYAN)
        draw.text((58, 104), segment.headline, font=_font(38, "bold"), fill=INK)
        if segment.body:
            draw.text((58, 162), segment.body, font=_font(19), fill=MUTED)
        if segment.rows:
            start_y = 250
            row_height = 92
            for row_index, row in enumerate(segment.rows):
                amount = _ease((progress - row_index * 0.12) / 0.24)
                color = _accent(row.accent)
                x2 = 1160
                y = start_y + row_index * (row_height + 18)
                draw.rounded_rectangle((86, y, x2, y + row_height), radius=18,
                                       fill=PANEL, outline=tuple(int(PANEL[j] + (color[j] - PANEL[j]) * amount)
                                                                 for j in range(3)), width=2)
                draw.text((116, y + 20), row.label.upper(), font=_font(15, "semi"), fill=color)
                draw.text((330, y + 19), row.detail, font=_font(20, "bold"), fill=INK)
        else:
            words = ["DESCRIBE", "INSPECT", "EDIT", "RUN", "MEASURE", "REFINE"]
            colors = [CYAN, BLUE, AMBER, GREEN, CYAN, BLUE]
            for item_index, (word, color) in enumerate(zip(words, colors)):
                x = 92 + item_index * 196
                amount = _ease((progress - item_index * 0.08) / 0.20)
                draw.rounded_rectangle((x, 320, x + 162, 432), radius=18, fill=PANEL,
                                       outline=tuple(int(PANEL[j] + (color[j] - PANEL[j]) * amount)
                                                     for j in range(3)), width=2)
                _centered(draw, (x + 81, 376), word, _font(15, "bold"), INK)
                if item_index < len(words) - 1:
                    draw.line((x + 168, 376, x + 188, 376), fill=(82, 96, 110), width=3)
        if segment.claim_boundary:
            _centered(draw, (PW // 2, 650), segment.claim_boundary.upper(), _font(16, "semi"), RED)
        return image
    output = out_dir / f"{index:03d}_{segment.id}.mp4"
    key = cache.key("plate", asdict(segment))
    label = f"segment:{index:03d}:{segment.id}"
    if not cache.hit(label, key, output):
        _render_raw_frames(output, segment.duration_s, painter, profile)
        cache.store(label, key, output)
    return output


def build_edl(spec: AgentBuildSpec) -> dict[str, Any]:
    """Return the complete direct-cut edit plan, including the locked intro."""
    entries: list[dict[str, Any]] = [
        {"index": 1, "id": "disclaimer", "beat": "intro", "master_in_s": 0.0,
         "master_out_s": 5.0, "duration_s": 5.0,
         "source": "MOTION_GRAPHIC:disclaimer", "purpose": "Silent AI-production disclosure.",
         "transition": "start"},
        {"index": 2, "id": "story_intro", "beat": "intro", "master_in_s": 5.0,
         "master_out_s": 10.0, "duration_s": 5.0,
         "source": "MOTION_GRAPHIC:story_intro", "purpose": "Silent Agent Build story signature.",
         "transition": "direct_cut"},
    ]
    if spec.opening == "action":
        entries = []
    cursor = spec.intro_duration_s
    for segment in spec.segments:
        source = segment.source or f"MOTION_GRAPHIC:{segment.id}"
        entries.append({
            "index": len(entries) + 1, "id": segment.id, "beat": segment.beat,
            "act": segment.act or None,
            "master_in_s": round(cursor, 3), "master_out_s": round(cursor + segment.duration_s, 3),
            "duration_s": segment.duration_s, "source": source,
            "source_in_s": segment.source_in_s if segment.source is not None else None,
            "source_out_s": (segment.source_in_s + segment.duration_s if segment.source is not None else None),
            "source_tail_hold_s": (segment.source_tail_hold_s
                                   if segment.kind == "clip" else None),
            "purpose": segment.purpose, "transition": "direct_cut",
            "replay": segment.replay, "replay_label": segment.replay_label,
            "claim_boundary": segment.claim_boundary,
        })
        cursor += segment.duration_s
    return {
        "version": 1, "style": STYLE_VERSION, "master_duration_s": spec.duration_s,
        "cuts_reviewed": len(entries) - 1, "transition_vocabulary": ["direct_cut"],
        "structure": spec.structure or "evidence_arc",
        "simulator_footage_ratio": round(spec.simulator_footage_ratio, 6),
        "climax_segment": spec.climax_segment,
        "entries": entries,
    }


def _write_edl(spec: AgentBuildSpec, out_dir: Path) -> tuple[Path, list[dict[str, Any]]]:
    payload = build_edl(spec)
    edl_path = out_dir / "edit_decision_list.json"
    edl_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return edl_path, payload["entries"]


def _concat(clips: list[Path], output: Path, profile: RenderProfile,
            cache: ArtifactCache, expected_duration_s: float) -> Path:
    key = cache.key("picture", {
        "timing_version": 2,
        "clips": [{"path": str(path), "sha256": _sha256(path)} for path in clips],
    })
    if cache.hit("picture", key, output):
        return output
    listing = output.with_suffix(".txt")
    # MP4 container durations can be rounded to milliseconds. Letting concat
    # infer those durations shifts every subsequent cut off the frame grid.
    listing.write_text("".join(
        f"file '{clip.resolve().as_posix()}'\n"
        f"duration {probe_media(clip)['frames'] / profile.fps:.9f}\n"
        for clip in clips
    ), encoding="utf-8")
    partial = output.with_name(output.stem + ".partial" + output.suffix)
    # Every part is normalized to the same H.264 profile, geometry, and CFR.
    # A packet-level concat makes revisions nearly instant.  If a platform's
    # ffmpeg build produces timing drift, fall back to the old full re-encode.
    _run([_ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
          "-f", "concat", "-safe", "0", "-i", str(listing), "-an",
          "-c:v", "copy", "-movflags", "+faststart", str(partial)])
    expected_frames = round(expected_duration_s * profile.fps)
    try:
        facts = probe_media(partial)
        duration_ok = abs(facts["duration_s"] - expected_duration_s) <= 1.1 / profile.fps
        frames_ok = facts["frames"] == expected_frames
        geometry_ok = (facts["width"], facts["height"]) == (profile.width, profile.height)
        rate_ok = abs(facts["fps"] - profile.fps) < 1e-9
    except Exception:
        duration_ok = frames_ok = geometry_ok = rate_ok = False
    if not all((duration_ok, frames_ok, geometry_ok, rate_ok)):
        partial.unlink(missing_ok=True)
        _run([
            _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
            "-f", "concat", "-safe", "0", "-i", str(listing), "-an",
            "-r", str(profile.fps), "-t", f"{expected_duration_s:.6f}",
            "-frames:v", str(expected_frames),
            "-c:v", "libx264", "-preset", profile.preset,
            "-crf", str(profile.crf), "-profile:v", "high",
            "-pix_fmt", "yuv420p", str(partial),
        ])
    partial.replace(output)
    cache.store("picture", key, output)
    return output


def _generate_score(spec: AgentBuildSpec, output: Path, cache: ArtifactCache,
                    sample_rate: int = 48000) -> Path:
    key = cache.key("score", {
        "duration_s": spec.duration_s,
        "opening": spec.opening,
        "action_score_version": 1,
        "segments": [{"id": segment.id, "duration_s": segment.duration_s}
                     for segment in spec.segments],
        "sample_rate": sample_rate,
    })
    if cache.hit("score", key, output):
        return output
    t = np.arange(int(spec.duration_s * sample_rate), dtype=np.float64) / sample_rate
    score = np.zeros_like(t)
    cues = [0.0]
    cursor = spec.intro_duration_s
    for segment in spec.segments:
        cues.append(cursor)
        cursor += segment.duration_s
    cues.append(cursor)
    chords = [(98.0, 146.83, 196.0), (110.0, 164.81, 220.0), (123.47, 185.0, 246.94)]
    for index, start in enumerate(cues):
        end = cues[index + 1] + 1.4 if index + 1 < len(cues) else spec.duration_s
        mask = (t >= start) & (t < end)
        local = t[mask] - start
        env = np.minimum(1.0, local / 2.4) * np.minimum(1.0, (end - t[mask]) / 1.8)
        pad = sum(np.sin(2 * np.pi * frequency * local + phase * 0.75)
                  for phase, frequency in enumerate(chords[index % len(chords)])) / 3.0
        score[mask] += env * pad * 0.020
    if spec.opening == "action":
        # Original, locally synthesized score: restrained harmonic bed and
        # pulse, with more movement during the uninterrupted decisive run.
        # Speech remains dominant through the mix's side-chain ducking.
        score = np.zeros_like(t)
        progression = [(130.81, 155.56, 196.0), (103.83, 130.81, 155.56),
                       (116.54, 146.83, 174.61), (98.0, 130.81, 155.56)]
        beat = 60.0 / 86.0
        climax_start = sum(s.duration_s for s in spec.segments[:next(
            (i for i, s in enumerate(spec.segments) if s.id == spec.climax_segment), 0)])
        for bar_start in np.arange(0.0, spec.duration_s, beat * 8):
            chord = progression[int(bar_start / (beat * 8)) % len(progression)]
            mask = (t >= bar_start) & (t < bar_start + beat * 8 + 1.5)
            local = t[mask] - bar_start
            env = np.minimum(1.0, local / .8) * np.clip((beat * 8 + 1.5 - local) / 1.6, 0, 1)
            score[mask] += .07 * env * sum(np.sin(2*np.pi*f*local) for f in chord) / 3
        for n, onset in enumerate(np.arange(0.0, spec.duration_s, beat)):
            mask = (t >= onset) & (t < onset + 1.2)
            local = t[mask] - onset
            chord = progression[int(onset / (beat * 8)) % len(progression)]
            frequency = chord[n % 3] * (2 if n % 2 else 1)
            strength = .055 if climax_start <= onset < spec.duration_s - 10 else .027
            score[mask] += strength * (1 - np.exp(-local*180)) * np.exp(-local*5.5) * np.sin(2*np.pi*frequency*local)
            if n % 2 == 0:
                score[mask] += .035 * np.exp(-local*16) * np.sin(2*np.pi*(48*local + 9*(1-np.exp(-local*18))))
        score *= np.clip((spec.duration_s - t) / 2.2, 0, 1)
    stereo = np.stack([score, np.roll(score, 1100) * 0.92], axis=1)
    pcm = (np.clip(stereo, -0.15, 0.15) * 32767).astype("<i2")
    with wave.open(str(output), "wb") as wav:
        wav.setnchannels(2)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm.tobytes())
    cache.store("score", key, output)
    return output


def _verify_voice_silence(path: Path, intro_duration_s: float = INTRO_DURATION_S) -> None:
    with wave.open(str(path), "rb") as wav:
        sample_rate = wav.getframerate()
        channels = wav.getnchannels()
        width = wav.getsampwidth()
        if width not in (2, 3, 4):
            raise ValueError("narration WAV must use integer PCM")
        prefix = wav.readframes(round(intro_duration_s * sample_rate))
    if any(prefix):
        raise ValueError("narration WAV contains voice/sample activity during the locked 0–10 second intro")
    if channels != 1:
        raise ValueError("narration WAV must be mono; the final mix is stereo")


def _mix(spec: AgentBuildSpec, picture: Path, voice: Path, score: Path, output: Path,
         profile: RenderProfile, cache: ArtifactCache) -> Path:
    _verify_voice_silence(voice, spec.intro_duration_s)
    key = cache.key("master_mix", {
        "picture_sha256": _sha256(picture), "voice_sha256": _sha256(voice),
        "score_sha256": _sha256(score), "intro_score_gain": INTRO_SCORE_GAIN,
        "story_score_gain": STORY_SCORE_GAIN, "intro_master_gain": INTRO_MASTER_GAIN,
        "intro_duration_s": spec.intro_duration_s,
    })
    if cache.hit("master_mix", key, output):
        return output
    audio_filter = (
        "[1:a]highpass=f=65,lowpass=f=15500,acompressor=threshold=0.14:ratio=1.65:attack=25:release=300,"
        "volume=1.05,asplit=2[voice_sc][voice_mix];"
        f"[2:a]volume='if(lt(t,{spec.intro_duration_s:g}),{INTRO_SCORE_GAIN:.3f},"
        f"{STORY_SCORE_GAIN:.3f})':eval=frame[bed];"
        "[bed][voice_sc]sidechaincompress=threshold=0.020:ratio=7:attack=18:release=560[duck];"
        "[duck][voice_mix]amix=inputs=2:duration=longest:normalize=0,"
        f"loudnorm=I=-15:TP=-1.5:LRA=8,volume='if(lt(t,{spec.intro_duration_s:g}),"
        f"{INTRO_MASTER_GAIN:.3f},1.0)':eval=frame[aout]"
    )
    _run([
        _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-i", str(picture), "-i", str(voice), "-i", str(score),
        "-filter_complex", audio_filter, "-map", "0:v", "-map", "[aout]",
        "-t", f"{spec.duration_s:.3f}",
        "-frames:v", str(round(spec.duration_s * profile.fps)),
        "-c:v", "copy", "-c:a", "aac", "-b:a", "256k", "-ar", "48000",
        "-movflags", "+faststart", str(output),
    ])
    cache.store("master_mix", key, output)
    return output


# A film without narration: the score is the whole soundtrack. It plays at the story gain the
# narrated mix gives its bed, with no speech to duck under, normalised to a music-only loudness
# a little below speech; the locked intro keeps its softer gain.
SCORE_ONLY_LOUDNESS_LUFS = -18.0


def _mix_score_only(spec: AgentBuildSpec, picture: Path, score: Path, output: Path,
                    profile: RenderProfile, cache: ArtifactCache) -> Path:
    key = cache.key("master_mix_score_only", {
        "picture_sha256": _sha256(picture), "score_sha256": _sha256(score),
        "intro_score_gain": INTRO_SCORE_GAIN, "story_score_gain": STORY_SCORE_GAIN,
        "intro_master_gain": INTRO_MASTER_GAIN, "loudness_lufs": SCORE_ONLY_LOUDNESS_LUFS,
        "intro_duration_s": spec.intro_duration_s,
    })
    if cache.hit("master_mix", key, output):
        return output
    audio_filter = (
        f"[1:a]volume='if(lt(t,{spec.intro_duration_s:g}),{INTRO_SCORE_GAIN:.3f},"
        f"{STORY_SCORE_GAIN:.3f})':eval=frame,"
        f"loudnorm=I={SCORE_ONLY_LOUDNESS_LUFS:g}:TP=-1.5:LRA=8,"
        f"volume='if(lt(t,{spec.intro_duration_s:g}),{INTRO_MASTER_GAIN:.3f},1.0)':eval=frame[aout]"
    )
    _run([
        _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-i", str(picture), "-i", str(score),
        "-filter_complex", audio_filter, "-map", "0:v", "-map", "[aout]",
        "-t", f"{spec.duration_s:.3f}",
        "-frames:v", str(round(spec.duration_s * profile.fps)),
        "-c:v", "copy", "-c:a", "aac", "-b:a", "256k", "-ar", "48000",
        "-movflags", "+faststart", str(output),
    ])
    cache.store("master_mix", key, output)
    return output


def render(spec: AgentBuildSpec, out_dir: Path | None = None, *,
           profile: RenderProfile = FINAL_PROFILE,
           preflight_checked: bool = False) -> dict[str, Path]:
    """Render an Agent Build manifest and emit its hash-bound production artifacts."""
    started = time.perf_counter()
    preflight_report = ({"valid": True, "reused_from_workflow_gate": True}
                        if preflight_checked else preflight(spec))
    base_out = (out_dir or spec.source_path.parent / "build" / spec.slug).resolve()
    out_dir = base_out if profile.name == "final" else base_out / profile.name
    parts = out_dir / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    cache = ArtifactCache(out_dir, profile)
    clips = list(render_locked_intro(parts, profile, cache)) if spec.opening == "signature" else []
    provenance_sources: list[dict[str, Any]] = []
    for index, segment in enumerate(spec.segments, 1):
        if segment.kind == "clip" or segment.source is not None:
            rendered = _render_clip(spec, segment, index, parts, profile, cache)
            source = _resolve(spec, segment.source or "")
            provenance_sources.append({
                "segment": segment.id, "source": str(source), "sha256": _sha256(source),
                "kind": segment.kind,
                "source_in_s": segment.source_in_s,
                "source_out_s": segment.source_in_s + segment.duration_s,
                "source_tail_hold_s": segment.source_tail_hold_s,
            })
        else:
            rendered = _render_plate(segment, index, parts, profile, cache)
        clips.append(rendered)
    edl_path, _ = _write_edl(spec, out_dir)
    provenance_path = out_dir / "capture_provenance.json"
    evidence_records: list[dict[str, str]] = []
    for value in spec.evidence:
        path = _resolve(spec, value)
        if not path.exists():
            raise FileNotFoundError(f"evidence artifact is missing: {path}")
        evidence_records.append({"path": str(path), "sha256": _sha256(path)})
    provenance_path.write_text(json.dumps({
        "version": 1, "style": STYLE_VERSION, "story": {
            "question": spec.question, "claim": spec.claim, "boundary": spec.boundary,
            "structure": spec.structure or "evidence_arc",
            "simulator_footage_ratio": round(spec.simulator_footage_ratio, 6),
            "climax_segment": spec.climax_segment,
        }, "captures": provenance_sources, "evidence": evidence_records,
    }, indent=2) + "\n", encoding="utf-8")
    picture = _concat(clips, out_dir / "picture.mp4", profile, cache, spec.duration_s)
    score = _generate_score(spec, out_dir / "score.wav", cache)
    master_name = f"{spec.slug}.mp4" if profile.name == "final" else f"{spec.slug}_proxy.mp4"
    artifacts = {
        "renderer": Path(__file__).resolve(),
        "manifest": spec.source_path,
        "edit_decision_list": edl_path, "capture_provenance": provenance_path,
    }
    if spec.voice.narrated:
        voice = _resolve(spec, spec.voice.wav)
        if not voice.exists():
            raise FileNotFoundError(f"narration WAV missing; run agent-build-voice first: {voice}")
        voice_script = _resolve(spec, spec.voice.script)
        voice_timings = voice.with_suffix(".segments.json")
        if not voice_script.exists() or not voice_timings.exists():
            raise FileNotFoundError("narration script/timing receipt missing; run agent-build-voice first")
        master = _mix(spec, picture, voice, score, out_dir / master_name, profile, cache)
        artifacts.update({
            "voice_generator": Path(__file__).with_name("agent_build_voice.py").resolve(),
            "narration_script": voice_script, "narration_timings": voice_timings, "narration_wav": voice,
        })
    else:
        master = _mix_score_only(spec, picture, score, out_dir / master_name, profile, cache)
    artifacts.update({"picture": picture, "score": score, "master": master})
    render_report = out_dir / "render_report.json"
    render_report.write_text(json.dumps({
        "version": 1, "style": STYLE_VERSION, "profile": asdict(profile),
        "elapsed_s": round(time.perf_counter() - started, 3),
        "cache": cache.report(), "preflight": preflight_report, "voice": _voice_record(spec),
    }, indent=2) + "\n", encoding="utf-8")
    artifacts["render_report"] = render_report
    receipt = out_dir / ("assembly_receipt.json" if profile.name == "final"
                         else "proxy_receipt.json")
    receipt.write_text(json.dumps({
        "version": 1, "style": STYLE_VERSION, "master_duration_s": spec.duration_s,
        "profile": asdict(profile),
        "video_frames": round(spec.duration_s * profile.fps),
        "voice": _voice_record(spec),
        "opening_contract": ({
            "disclosure": [0.0, 5.0], "story_signature": [5.0, 10.0],
            "first_build_footage_s": 10.0, "voiceover_present_during_intro": False,
        } if spec.opening == "signature" else {
            "opening": "action", "first_build_footage_s": 0.0,
            "disclosure": "Overlay on the first simulator shot", "story_signature": None,
        }),
        "artifacts": {name: {"path": str(path), "sha256": _sha256(path)}
                      for name, path in artifacts.items()},
    }, indent=2) + "\n", encoding="utf-8")
    return {"output_dir": out_dir, "master": master, "edl": edl_path,
            "provenance": provenance_path, "receipt": receipt,
            "render_report": render_report}


def _mean_db(path: Path, start: float, duration: float) -> float:
    out = subprocess.run([_ffmpeg(), "-hide_banner", "-ss", f"{start:.3f}", "-t", f"{duration:.3f}",
                          "-i", str(path), "-vn", "-af", "volumedetect", "-f", "null", "-"],
                         capture_output=True, text=True).stderr
    match = re.search(r"mean_volume:\s*(-?[\d.]+|-inf) dB", out)
    return float(match.group(1)) if match and match.group(1) != "-inf" else -120.0


def _measure_intro_vs_story(master: Path, spec: AgentBuildSpec) -> dict[str, Any]:
    """The delivered master's locked intro against the story window after it (AGENT_BUILD_FILMS.md,
    Audio hierarchy: at least 3 dB below)."""
    if spec.intro_duration_s <= 0:
        return {"intro_minus_story_db": None, "note": "action opening: no locked intro"}
    window = min(30.0, spec.content_duration_s)
    intro = _mean_db(master, 0.0, spec.intro_duration_s)
    story = _mean_db(master, spec.intro_duration_s, window)
    return {"intro_mean_db": intro, "story_mean_db": story, "intro_minus_story_db": round(intro - story, 2),
            "story_window_s": [spec.intro_duration_s, spec.intro_duration_s + window]}


def verify(spec: AgentBuildSpec, out_dir: Path | None = None) -> dict[str, Any]:
    """Fail closed on the locked intro, story contract, hashes, and delivery."""
    out_dir = (out_dir or spec.source_path.parent / "build" / spec.slug).resolve()
    master = out_dir / f"{spec.slug}.mp4"
    edl_path = out_dir / "edit_decision_list.json"
    receipt_path = out_dir / "assembly_receipt.json"
    for path in (master, edl_path, receipt_path):
        if not path.exists():
            raise FileNotFoundError(path)
    edl = json.loads(edl_path.read_text(encoding="utf-8"))
    entries = edl.get("entries", [])
    expected = [
        ("disclaimer", "MOTION_GRAPHIC:disclaimer", 0.0, 5.0),
        ("story_intro", "MOTION_GRAPHIC:story_intro", 5.0, 10.0),
    ]
    if spec.opening == "action":
        expected = []
    for index, (clip_id, source, start, end) in enumerate(expected):
        entry = entries[index]
        if (entry.get("id"), entry.get("source"), float(entry.get("master_in_s")),
                float(entry.get("master_out_s"))) != (clip_id, source, start, end):
            raise AssertionError(f"locked opening contract drifted at {clip_id}")
    first = entries[len(expected)]
    if first.get("id") != spec.segments[0].id or float(first["master_in_s"]) != spec.intro_duration_s:
        raise AssertionError("real build footage does not begin at the declared story start")
    if any(entry.get("id") == "outro" or entry.get("source") == "MOTION_GRAPHIC:locked_outro"
           for entry in entries):
        raise AssertionError("Agent Build masters must not contain an outro")
    if (entries[-1].get("id") != spec.segments[-1].id
            or abs(float(entries[-1]["master_out_s"]) - spec.duration_s) > 0.001):
        raise AssertionError("the final story segment must end the master")

    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    for record in receipt.get("artifacts", {}).values():
        path = Path(record["path"])
        if not path.exists() or _sha256(path) != record["sha256"]:
            raise AssertionError(f"receipt-bound artifact changed: {path}")
    if spec.voice.narrated:
        _verify_voice_silence(_resolve(spec, spec.voice.wav), spec.intro_duration_s)
    elif receipt.get("voice", {}).get("mode") != "none" or "narration_wav" in receipt.get("artifacts", {}):
        raise AssertionError("the manifest declares no narration but the assembly receipt does not agree")

    probe = json.loads(subprocess.run([
        _ffprobe(), "-v", "error", "-show_entries",
        "format=duration:stream=codec_name,profile,codec_type,width,height,pix_fmt,"
        "r_frame_rate,avg_frame_rate,sample_rate,channels,duration,nb_frames",
        "-of", "json", str(master),
    ], capture_output=True, text=True, check=True).stdout)
    video = next(stream for stream in probe["streams"] if stream["codec_type"] == "video")
    audio = next(stream for stream in probe["streams"] if stream["codec_type"] == "audio")
    if any(stream["codec_type"] == "subtitle" for stream in probe["streams"]):
        raise AssertionError("Agent Build masters never carry a subtitle stream")
    if (video["codec_name"], video["profile"], int(video["width"]), int(video["height"]),
            video["pix_fmt"], video["r_frame_rate"], video["avg_frame_rate"]) != (
            "h264", "High", W, H, "yuv420p", "30/1", "30/1"):
        raise AssertionError("video delivery contract failed")
    if int(video["nb_frames"]) != round(spec.duration_s * FPS):
        raise AssertionError("frame count does not match the manifest")
    if audio["codec_name"] != "aac" or audio["sample_rate"] != "48000" or int(audio["channels"]) != 2:
        raise AssertionError("audio delivery contract failed")
    if abs(float(probe["format"]["duration"]) - spec.duration_s) > 0.001:
        raise AssertionError("master duration does not match the manifest")

    report = {
        "approved": True, "style": STYLE_VERSION, "master": str(master),
        "master_sha256": _sha256(master), "duration_s": spec.duration_s,
        "frames": round(spec.duration_s * FPS), "intro_voiceover": spec.opening == "action",
        "opening": spec.opening,
        "disclosure_s": ([0.0, 5.0] if spec.opening == "signature" else
                         [0.0, spec.segments[0].duration_s] if spec.on_screen_text == "full" else None),
        "on_screen_text": spec.on_screen_text,
        "off_screen_texts": ({"disclosure": DISCLOSURE,
                              "claim_boundary": next((s.claim_boundary for s in spec.segments
                                                      if s.claim_boundary), "")}
                             if spec.on_screen_text == "none" else None),
        "story_signature_s": [5.0, 10.0] if spec.opening == "signature" else None,
        "first_build_footage_s": spec.intro_duration_s, "subtitle_streams": 0,
        "cuts": len(entries) - 1, "outro": False,
        "voice": _voice_record(spec),
        "audio_hierarchy": _measure_intro_vs_story(master, spec),
    }
    if (not spec.voice.narrated and spec.opening == "signature"
            and report["audio_hierarchy"]["intro_minus_story_db"] > -3.0):
        raise AssertionError("the locked intro must sit at least 3 dB below the story soundtrack")
    (out_dir / "verification.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
