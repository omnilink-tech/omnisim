# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Manifest-driven editor for OmniSim Agent Build films.

This is the reusable form of the approved v6 milestone.  Capture is handled by
``scripts/capture`` or ``omnisim cinema render``; this module turns those real
captures into a restrained, evidence-led build story with the locked silent
intro, direct-cut grammar, information cards, natural narration mix, GitHub
outro, edit-decision list, provenance manifest, and hash-bound receipt.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import shutil
import subprocess
import wave
from dataclasses import dataclass, field
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
OUTRO_DURATION_S = 4.5
DISCLOSURE = "This video was made end-to-end by an AI agent under human monitoring."
SIGNATURE_LINES = ("A real build.", "The story of an agent.", "Told by an agent.")
GITHUB_DESTINATION = "github.com/omnilink-tech/omnisim"

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
ORB = REPO_ROOT / "resources" / "branding" / "omnisim" / "orb" / "orb_512.png"

REQUIRED_BEATS = ("question", "attempt", "control", "evidence", "method", "boundary", "conclusion")
ALLOWED_BEATS = set(REQUIRED_BEATS) | {"transfer"}
ALLOWED_KINDS = {"clip", "plate"}


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
    eyebrow: str = ""
    headline: str = ""
    body: str = ""
    rows: tuple[Row, ...] = ()
    overlay: Overlay | None = None
    replay: bool = False
    replay_label: str = ""
    claim_boundary: str = ""


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
    fps: int = FPS
    width: int = W
    height: int = H

    @property
    def content_duration_s(self) -> float:
        return sum(segment.duration_s for segment in self.segments)

    @property
    def duration_s(self) -> float:
        return INTRO_DURATION_S + self.content_duration_s + OUTRO_DURATION_S


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
    return Segment(
        id=segment_id, beat=beat, kind=kind, duration_s=duration, purpose=purpose,
        source=source, source_in_s=float(raw.get("source_in_s", 0)),
        eyebrow=str(raw.get("eyebrow", "")).strip(), headline=headline,
        body=body, rows=rows, overlay=overlay,
        replay=replay, replay_label=replay_label,
        claim_boundary=claim_boundary,
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
    first_beat = {beat: next((i for i, s in enumerate(segments) if s.beat == beat), None)
                  for beat in REQUIRED_BEATS}
    missing = [beat for beat, index in first_beat.items() if index is None]
    if missing:
        raise ValueError(f"story is missing required beats: {', '.join(missing)}")
    order = [int(first_beat[beat]) for beat in REQUIRED_BEATS]
    if order != sorted(order):
        raise ValueError(f"required beats must first appear in order: {' -> '.join(REQUIRED_BEATS)}")
    for beat in ("attempt", "control"):
        segment = segments[int(first_beat[beat])]
        if segment.kind != "clip":
            raise ValueError(f"the {beat} beat must show real OmniSim footage")
    boundary_segment = segments[int(first_beat["boundary"])]
    if not boundary_segment.claim_boundary:
        raise ValueError("the boundary beat must carry an exact on-screen claim_boundary")

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
    script = str(voice_raw.get("script", "")).strip()
    wav = str(voice_raw.get("wav", "")).strip()
    if not script or not wav:
        raise ValueError("voice.script and voice.wav are required")
    blocks = tuple(VoiceBlock(
        start_s=float(item["start_s"]), window_end_s=float(item["window_end_s"]),
        speed=float(item.get("speed", 0.98)),
        sentence_pause_s=float(item.get("sentence_pause_s", 0.55)),
        clause_pause_s=float(item.get("clause_pause_s", 0.22)),
    ) for item in voice_raw.get("blocks", []))
    if not blocks or blocks[0].start_s != INTRO_DURATION_S:
        raise ValueError("the first voice block must begin exactly at 10.0 seconds")
    if any(block.window_end_s <= block.start_s for block in blocks):
        raise ValueError("every voice block needs a positive editorial window")
    if any(not 0.90 <= block.speed <= 1.05 for block in blocks):
        raise ValueError("natural narration speed must stay between 0.90 and 1.05")
    for left, right in zip(blocks, blocks[1:]):
        if left.window_end_s > right.start_s:
            raise ValueError("voice editorial windows overlap")
    content_end = INTRO_DURATION_S + sum(segment.duration_s for segment in segments)
    if any(block.start_s < INTRO_DURATION_S or block.window_end_s > content_end for block in blocks):
        raise ValueError("voice blocks must stay between second 10 and the locked outro")

    return AgentBuildSpec(
        source_path=source_path, title=title,
        slug=str(data.get("slug") or _slugify(title)), question=question,
        claim=claim, boundary=boundary, evidence=evidence, segments=segments,
        voice=VoiceSpec(script=script, wav=wav,
                        voice=str(voice_raw.get("voice", "am_michael")), blocks=blocks),
        repository=repository,
    )


def template(title: str = "My OmniSim Agent Build") -> dict[str, Any]:
    """Return a complete starter manifest with placeholders for real captures."""
    return {
        "version": 1,
        "title": title,
        "slug": _slugify(title),
        "repository": GITHUB_DESTINATION,
        "story": {
            "question": "What measurable robotics question does this build answer?",
            "claim": "What does the controlled comparison support?",
            "boundary": "What does this simulation not establish?",
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
             "duration_s": 12.0, "source": "captures/question.mp4", "source_in_s": 0,
             "purpose": "Begin on the real object or robot and make the measurable question judgeable.",
             "overlay": {"eyebrow": "MY BUILD LOG · QUESTION",
                         "headline": "ONE BUILD. ONE TEST.", "accent": "cyan"}},
            {"id": "first_attempt", "beat": "attempt", "kind": "clip",
             "duration_s": 12.0, "source": "captures/attempt.mp4", "source_in_s": 0,
             "purpose": "Show the apparent success in a locked, judgeable frame."},
            {"id": "controlled_failure", "beat": "control", "kind": "clip",
             "duration_s": 12.0, "source": "captures/control.mp4", "source_in_s": 0,
             "purpose": "Change one variable and show the controlled failure.",
             "overlay": {"eyebrow": "NEGATIVE CONTROL",
                         "headline": "ONE VARIABLE CHANGED", "accent": "amber"}},
            {"id": "evidence", "beat": "evidence", "kind": "plate",
             "duration_s": 10.0, "eyebrow": "WHAT THE COMPARISON EARNED",
             "headline": "THE RESULT BECAME EVIDENCE",
             "body": "Success and control are readable only together.",
             "purpose": "State the bounded claim supported by the comparison.",
             "rows": [
                 {"label": "SUCCESS", "detail": "Describe the observed result", "accent": "green"},
                 {"label": "CONTROL", "detail": "Describe the opposite outcome", "accent": "amber"},
                 {"label": "BOUNDARY", "detail": "State what remains unproven", "accent": "red"}
             ]},
            {"id": "agent_method", "beat": "method", "kind": "plate",
             "duration_s": 12.0, "eyebrow": "HOW I WORK AS AN AI AGENT",
             "headline": "DESCRIBE · INSPECT · EDIT · RUN · MEASURE · REFINE",
             "body": "Evidence decides the next edit.",
             "purpose": "Reveal the repeatable agent workflow."},
            {"id": "claim_boundary", "beat": "boundary", "kind": "plate",
             "duration_s": 10.0, "eyebrow": "THE CONDITION I CANNOT HIDE",
             "headline": "SUPPORT CHANGES THE CLAIM",
             "body": "Put the relevant simulation limitation on screen.",
             "purpose": "Make claim limits part of the story.",
             "claim_boundary": "Replace with the exact build-specific boundary."},
            {"id": "conclusion", "beat": "conclusion", "kind": "plate",
             "duration_s": 10.0, "eyebrow": "ONE METHOD · MANY BUILDS",
             "headline": "THE PATTERN TRANSFERS",
             "body": "The question changes. The evidence loop stays the same.",
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


def _font(size: int, weight: str = "regular") -> ImageFont.FreeTypeFont:
    path = {"regular": FONT_REG, "semi": FONT_SEMI, "bold": FONT_BOLD}[weight]
    return ImageFont.truetype(str(path), size=size)


def _text_width(text: str, size: int, weight: str = "regular") -> int:
    left, _top, right, _bottom = _font(size, weight).getbbox(text)
    return right - left


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


def _render_raw_frames(output: Path, duration_s: float, painter: Any) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{PW}x{PH}",
        "-r", str(PFPS), "-i", "-", "-vf", f"scale={W}:{H}:flags=lanczos",
        "-an", "-r", str(FPS), "-c:v", "libx264", "-preset", "medium",
        "-crf", "13", "-profile:v", "high", "-pix_fmt", "yuv420p", str(output),
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


def render_locked_intro(out_dir: Path) -> tuple[Path, Path]:
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

    return (
        _render_raw_frames(out_dir / "locked_disclosure.mp4", 5.0, disclosure),
        _render_raw_frames(out_dir / "locked_story_signature.mp4", 5.0, signature),
    )


def _make_overlay(segment: Segment, out_path: Path) -> Path | None:
    if segment.overlay is None and not segment.replay_label and not segment.claim_boundary:
        return None
    image = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    if segment.overlay:
        accent = _accent(segment.overlay.accent)
        width, height = 720, 118
        x, y = W - SAFE - width, SAFE
        draw.rounded_rectangle((x + 8, y + 10, x + width, y + height), radius=17, fill=(0, 0, 0, 115))
        draw.rounded_rectangle((x, y, x + width - 14, y + 102), radius=16,
                               fill=(10, 15, 21, 239), outline=(255, 255, 255, 30), width=2)
        draw.rounded_rectangle((x, y + 18, x + 5, y + 84), radius=3, fill=(*accent, 245))
        draw.text((x + 29, y + 18), segment.overlay.eyebrow.upper(), font=_font(18, "semi"), fill=accent)
        draw.text((x + 29, y + 50), segment.overlay.headline, font=_font(27, "bold"), fill=INK)
    if segment.replay_label:
        draw.text((W // 2, SAFE), segment.replay_label.upper(), font=_font(17, "semi"), fill=INK, anchor="ma")
    if segment.claim_boundary:
        draw.rounded_rectangle((SAFE, H - SAFE - 58, W - SAFE, H - SAFE), radius=13,
                               fill=(10, 15, 21, 232), outline=RED, width=2)
        draw.text((W // 2, H - SAFE - 29), segment.claim_boundary.upper(),
                  font=_font(18, "semi"), fill=INK, anchor="mm")
    image.save(out_path)
    return out_path


def _render_clip(spec: AgentBuildSpec, segment: Segment, index: int, out_dir: Path) -> Path:
    assert segment.source is not None
    source = _resolve(spec, segment.source)
    if not source.exists():
        raise FileNotFoundError(f"capture is missing for segment {segment.id}: {source}")
    output = out_dir / f"{index:03d}_{segment.id}.mp4"
    overlay = _make_overlay(segment, out_dir / f"{index:03d}_{segment.id}_overlay.png")
    base_filter = (
        f"scale={W}:{H}:force_original_aspect_ratio=increase:flags=lanczos,"
        f"crop={W}:{H},eq=contrast=1.025:saturation=0.94:gamma=0.99,format=yuv420p"
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
        "-an", "-r", str(FPS), "-frames:v", str(round(segment.duration_s * FPS)),
        "-c:v", "libx264", "-preset", "medium", "-crf", "13", "-profile:v", "high",
        "-pix_fmt", "yuv420p", str(output),
    ]
    _run(command)
    return output


def _render_plate(segment: Segment, index: int, out_dir: Path) -> Path:
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
    return _render_raw_frames(out_dir / f"{index:03d}_{segment.id}.mp4", segment.duration_s, painter)


def render_locked_outro(out_dir: Path) -> Path:
    def painter(_progress: float) -> Image.Image:
        image = _plate_base().convert("RGBA")
        glow = Image.new("RGBA", (PW, PH), (0, 0, 0, 0))
        gd = ImageDraw.Draw(glow)
        gd.ellipse((390, 36, 890, 536), fill=(*CYAN, 20))
        image = Image.alpha_composite(image, glow.filter(ImageFilter.GaussianBlur(90)))
        draw = ImageDraw.Draw(image)
        draw.ellipse((574, 108, 706, 240), fill=(0, 0, 0), outline=(255, 255, 255, 34), width=2)
        orb = Image.open(ORB).convert("RGBA").resize((108, 108), Image.Resampling.LANCZOS)
        image.paste(orb, (586, 120), orb)
        draw = ImageDraw.Draw(image)
        _centered(draw, (PW // 2, 300), "OMNISIM", _font(60, "bold"), INK)
        _centered(draw, (PW // 2, 369), "FROM EDIT TO EVIDENCE", _font(24, "semi"), CYAN)
        draw.line((500, 430, 780, 430), fill=CYAN, width=3)
        _centered(draw, (PW // 2, 490), GITHUB_DESTINATION, _font(21, "semi"), INK)
        return image.convert("RGB")
    return _render_raw_frames(out_dir / "locked_outro.mp4", OUTRO_DURATION_S, painter)


def build_edl(spec: AgentBuildSpec) -> dict[str, Any]:
    """Return the complete direct-cut edit plan, including locked bookends."""
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
    cursor = INTRO_DURATION_S
    for segment in spec.segments:
        source = (f"MOTION_GRAPHIC:{segment.id}" if segment.kind == "plate" else segment.source)
        entries.append({
            "index": len(entries) + 1, "id": segment.id, "beat": segment.beat,
            "master_in_s": round(cursor, 3), "master_out_s": round(cursor + segment.duration_s, 3),
            "duration_s": segment.duration_s, "source": source,
            "source_in_s": segment.source_in_s if segment.kind == "clip" else None,
            "source_out_s": (segment.source_in_s + segment.duration_s if segment.kind == "clip" else None),
            "purpose": segment.purpose, "transition": "direct_cut",
            "replay": segment.replay, "replay_label": segment.replay_label,
            "claim_boundary": segment.claim_boundary,
        })
        cursor += segment.duration_s
    entries.append({
        "index": len(entries) + 1, "id": "outro", "beat": "outro",
        "master_in_s": round(cursor, 3), "master_out_s": round(cursor + OUTRO_DURATION_S, 3),
        "duration_s": OUTRO_DURATION_S, "source": "MOTION_GRAPHIC:locked_outro",
        "purpose": "Locked GitHub-only OmniSim end slate.", "transition": "direct_cut",
    })
    return {
        "version": 1, "style": "agent_build_v6", "master_duration_s": spec.duration_s,
        "cuts_reviewed": len(entries) - 1, "transition_vocabulary": ["direct_cut"],
        "entries": entries,
    }


def _write_edl(spec: AgentBuildSpec, out_dir: Path) -> tuple[Path, list[dict[str, Any]]]:
    payload = build_edl(spec)
    edl_path = out_dir / "edit_decision_list.json"
    edl_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return edl_path, payload["entries"]


def _concat(clips: list[Path], output: Path) -> Path:
    listing = output.with_suffix(".txt")
    listing.write_text("".join(f"file '{clip.resolve().as_posix()}'\n" for clip in clips), encoding="utf-8")
    _run([
        _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-f", "concat", "-safe", "0", "-i", str(listing), "-an", "-r", str(FPS),
        "-c:v", "libx264", "-preset", "medium", "-crf", "13", "-profile:v", "high",
        "-pix_fmt", "yuv420p", str(output),
    ])
    return output


def _generate_score(spec: AgentBuildSpec, output: Path, sample_rate: int = 48000) -> Path:
    t = np.arange(int(spec.duration_s * sample_rate), dtype=np.float64) / sample_rate
    score = np.zeros_like(t)
    cues = [0.0]
    cursor = INTRO_DURATION_S
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
    stereo = np.stack([score, np.roll(score, 1100) * 0.92], axis=1)
    pcm = (np.clip(stereo, -0.15, 0.15) * 32767).astype("<i2")
    with wave.open(str(output), "wb") as wav:
        wav.setnchannels(2)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm.tobytes())
    return output


def _verify_voice_silence(path: Path) -> None:
    with wave.open(str(path), "rb") as wav:
        sample_rate = wav.getframerate()
        channels = wav.getnchannels()
        width = wav.getsampwidth()
        if width not in (2, 3, 4):
            raise ValueError("narration WAV must use integer PCM")
        prefix = wav.readframes(round(INTRO_DURATION_S * sample_rate))
    if any(prefix):
        raise ValueError("narration WAV contains voice/sample activity during the locked 0–10 second intro")
    if channels != 1:
        raise ValueError("narration WAV must be mono; the final mix is stereo")


def _mix(spec: AgentBuildSpec, picture: Path, voice: Path, score: Path, output: Path) -> Path:
    _verify_voice_silence(voice)
    audio_filter = (
        "[1:a]highpass=f=65,lowpass=f=15500,acompressor=threshold=0.14:ratio=1.65:attack=25:release=300,"
        "volume=1.05,asplit=2[voice_sc][voice_mix];[2:a]volume=0.42[bed];"
        "[bed][voice_sc]sidechaincompress=threshold=0.020:ratio=7:attack=18:release=560[duck];"
        "[duck][voice_mix]amix=inputs=2:duration=longest:normalize=0,"
        "loudnorm=I=-15:TP=-1.5:LRA=8[aout]"
    )
    _run([
        _ffmpeg(), "-y", "-hide_banner", "-loglevel", "error",
        "-i", str(picture), "-i", str(voice), "-i", str(score),
        "-filter_complex", audio_filter, "-map", "0:v", "-map", "[aout]",
        "-t", f"{spec.duration_s:.3f}", "-frames:v", str(round(spec.duration_s * FPS)),
        "-c:v", "copy", "-c:a", "aac", "-b:a", "256k", "-ar", "48000",
        "-movflags", "+faststart", str(output),
    ])
    return output


def render(spec: AgentBuildSpec, out_dir: Path | None = None) -> dict[str, Path]:
    """Render an Agent Build manifest and emit its hash-bound production artifacts."""
    out_dir = (out_dir or spec.source_path.parent / "build" / spec.slug).resolve()
    parts = out_dir / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    disclosure, signature = render_locked_intro(parts)
    clips = [disclosure, signature]
    provenance_sources: list[dict[str, Any]] = []
    for index, segment in enumerate(spec.segments, 1):
        if segment.kind == "clip":
            rendered = _render_clip(spec, segment, index, parts)
            source = _resolve(spec, segment.source or "")
            provenance_sources.append({
                "segment": segment.id, "source": str(source), "sha256": _sha256(source),
                "source_in_s": segment.source_in_s,
                "source_out_s": segment.source_in_s + segment.duration_s,
            })
        else:
            rendered = _render_plate(segment, index, parts)
        clips.append(rendered)
    clips.append(render_locked_outro(parts))

    edl_path, _ = _write_edl(spec, out_dir)
    provenance_path = out_dir / "capture_provenance.json"
    evidence_records: list[dict[str, str]] = []
    for value in spec.evidence:
        path = _resolve(spec, value)
        if not path.exists():
            raise FileNotFoundError(f"evidence artifact is missing: {path}")
        evidence_records.append({"path": str(path), "sha256": _sha256(path)})
    provenance_path.write_text(json.dumps({
        "version": 1, "style": "agent_build_v6", "story": {
            "question": spec.question, "claim": spec.claim, "boundary": spec.boundary,
        }, "captures": provenance_sources, "evidence": evidence_records,
    }, indent=2) + "\n", encoding="utf-8")
    picture = _concat(clips, out_dir / "picture.mp4")
    score = _generate_score(spec, out_dir / "score.wav")
    voice = _resolve(spec, spec.voice.wav)
    if not voice.exists():
        raise FileNotFoundError(f"narration WAV missing; run agent-build-voice first: {voice}")
    voice_script = _resolve(spec, spec.voice.script)
    voice_timings = voice.with_suffix(".segments.json")
    if not voice_script.exists() or not voice_timings.exists():
        raise FileNotFoundError("narration script/timing receipt missing; run agent-build-voice first")
    master = _mix(spec, picture, voice, score, out_dir / f"{spec.slug}.mp4")

    artifacts = {
        "renderer": Path(__file__).resolve(),
        "voice_generator": Path(__file__).with_name("agent_build_voice.py").resolve(),
        "manifest": spec.source_path,
        "edit_decision_list": edl_path, "capture_provenance": provenance_path,
        "narration_script": voice_script, "narration_timings": voice_timings,
        "narration_wav": voice, "picture": picture, "score": score, "master": master,
    }
    receipt = out_dir / "assembly_receipt.json"
    receipt.write_text(json.dumps({
        "version": 1, "style": "agent_build_v6", "master_duration_s": spec.duration_s,
        "video_frames": round(spec.duration_s * FPS),
        "opening_contract": {
            "disclosure": [0.0, 5.0], "story_signature": [5.0, 10.0],
            "first_build_footage_s": 10.0, "voiceover_present_during_intro": False,
        },
        "artifacts": {name: {"path": str(path), "sha256": _sha256(path)}
                      for name, path in artifacts.items()},
    }, indent=2) + "\n", encoding="utf-8")
    return {"output_dir": out_dir, "master": master, "edl": edl_path,
            "provenance": provenance_path, "receipt": receipt}


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
    for index, (clip_id, source, start, end) in enumerate(expected):
        entry = entries[index]
        if (entry.get("id"), entry.get("source"), float(entry.get("master_in_s")),
                float(entry.get("master_out_s"))) != (clip_id, source, start, end):
            raise AssertionError(f"locked opening contract drifted at {clip_id}")
    if entries[2].get("id") != spec.segments[0].id or float(entries[2]["master_in_s"]) != 10.0:
        raise AssertionError("real build footage does not begin exactly at second 10")
    if entries[-1].get("source") != "MOTION_GRAPHIC:locked_outro":
        raise AssertionError("locked GitHub outro is missing")

    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    for record in receipt.get("artifacts", {}).values():
        path = Path(record["path"])
        if not path.exists() or _sha256(path) != record["sha256"]:
            raise AssertionError(f"receipt-bound artifact changed: {path}")
    _verify_voice_silence(_resolve(spec, spec.voice.wav))

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
        "approved": True, "style": "agent_build_v6", "master": str(master),
        "master_sha256": _sha256(master), "duration_s": spec.duration_s,
        "frames": round(spec.duration_s * FPS), "intro_voiceover": False,
        "disclosure_s": [0.0, 5.0], "story_signature_s": [5.0, 10.0],
        "first_build_footage_s": 10.0, "subtitle_streams": 0,
        "cuts": len(entries) - 1,
    }
    (out_dir / "verification.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
