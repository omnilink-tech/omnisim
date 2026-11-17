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
"""Cut the warehouse-shift film from a take, in 16:9 (wide) and 9:16 (tall).

    python agents/omnilink_demos/warehouse_shift/film/edit.py <take_dir> <beats.json> <out_prefix>

beats.json lists the beats in order:
  {"step": "<shift step id>",
   "prompt": "<card text: the operator's sentence, abridged with an ellipsis if needed>",
   "answer": "<faithful excerpt of the robot's real reply, or null>",
   "from": <s before/after the step fired>, "to": <s after it fired, or "reply+<s>">,
   "speed": <simulated seconds per output second>}

Every output frame is a recorded native frame (frames are named by simulator
milliseconds; the take's rows.jsonl gives each step's fire and reply times on
the same clock). Edits are written to <out_prefix>_notes.json: per beat the
time window, the speed-up, and any freeze used to hold a card for reading.
Style: scripts/cinema/OMNILINK_DEMOS.md (signature, one card, silent).
"""
import gzip
import json
import shutil
import subprocess
import sys
import tempfile
from bisect import bisect_right
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[3] / "scripts" / "cinema" / "styles"))
import scene  # noqa: E402
from omnilink_minimal import overlay  # noqa: E402

FPS = 24
PROMPT_S = 2.6          # output seconds a prompt card stays up
ANSWER_S = 3.2          # output seconds an answer card stays up
FONT = "C:/Windows/Fonts/segoeui.ttf"
FONT_BOLD = "C:/Windows/Fonts/segoeuib.ttf"
FOOTNOTE = "Simulated in OmniSim · a real 30-minute OmniLink run with Claude Opus 5.5, time-compressed"
EXPORT = {"wide": (1920, 1080), "tall": (1080, 1920)}


def _font(n):
    return ImageFont.truetype(FONT, max(8, round(n)))


def _wrap(draw, text, f, width):
    lines = []
    for paragraph in text.split("\n"):
        line = ""
        for word in paragraph.split():
            cand = (line + " " + word).strip()
            if line and draw.textlength(cand, font=f) > width:
                lines.append(line)
                line = word
            else:
                line = cand
        if line:
            lines.append(line)
    return lines


def tall_overlay(frame, prompt=None, answer=None, robot="HUSKY"):
    """The minimal style for 9:16: the card sits in the open space above the floor."""
    s = frame.width / 900
    layer = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    d.text((round(30 * s), round(30 * s)), "OmniLink", font=_font(26 * s), fill=(245, 247, 249, 235))
    text = answer if answer is not None else prompt
    if text:
        f = _font((30 if answer is None else 26) * s)
        lines = _wrap(d, text, f, 760 * s)
        if len(lines) > 4:
            raise ValueError(f"Card too long for the tall cut: {text!r}")
        lh = round((42 if answer is None else 36) * s)
        label_h = round(24 * s) if answer is not None else 0
        width = round(max(d.textlength(line, font=f) for line in lines) + 56 * s)
        height = lh * len(lines) + round(34 * s) + label_h
        left, top = (frame.width - width) // 2, round(170 * s)
        d.rounded_rectangle((left, top, left + width, top + height), radius=round(18 * s), fill=(12, 20, 29, 235))
        y = top + round(14 * s)
        if answer is not None:
            d.text((left + round(28 * s), y - round(3 * s)), robot, font=_font(16 * s), fill=(111, 231, 208, 255))
            y += label_h
        for line in lines:
            d.text((left + round(28 * s), y), line, font=f, fill=(246, 248, 250, 255))
            y += lh
    return Image.alpha_composite(frame.convert("RGBA"), layer).convert("RGB")


def labels(frame, cam):
    """Small floor signage for the four stations (projected from the scene's camera model)."""
    _, _, _, w, h = scene.CAMERAS[cam]
    sx, sy = frame.width / w, frame.height / h
    layer = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    s = frame.width / (1600 if cam == "wide" else 900)
    f = ImageFont.truetype(FONT_BOLD, max(8, round(15 * s)))
    for name, (x, y) in scene.STATIONS.items():
        u, v = scene.project(cam, (x, y - 0.75, 0.0))
        text = name.upper()
        tw = d.textlength(text, font=f)
        cx, top = u * sx, v * sy
        pad_x, pad_y = 8 * s, 4 * s
        d.rounded_rectangle((cx - tw / 2 - pad_x, top - pad_y, cx + tw / 2 + pad_x, top + 19 * s + pad_y),
                            radius=round(6 * s), fill=(12, 20, 29, 200))
        d.text((cx - tw / 2, top), text, font=f, fill=(246, 248, 250, 255))
    return Image.alpha_composite(frame.convert("RGBA"), layer).convert("RGB")


def chip(frame, text):
    """A small time marker under the signature, e.g. '14 minutes later'."""
    s = frame.width / (1600 if frame.width > frame.height else 900)
    layer = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    f = ImageFont.truetype(FONT_BOLD, max(8, round(18 * s)))
    tw = d.textlength(text, font=f)
    x, y = round(36 * s), round(72 * s)
    d.rounded_rectangle((x - 12 * s, y - 6 * s, x + tw + 12 * s, y + 26 * s), radius=round(8 * s), fill=(12, 20, 29, 215))
    d.text((x, y - 1 * s), text, font=f, fill=(111, 231, 208, 255))
    return Image.alpha_composite(frame.convert("RGBA"), layer).convert("RGB")


def footnote(frame):
    layer = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    s = frame.width / (1600 if frame.width > frame.height else 900)
    f = _font(15 * s)
    lines = _wrap(d, FOOTNOTE, f, frame.width - 60 * s)
    y = frame.height - round(26 * s) - len(lines) * round(21 * s)
    for line in lines:
        d.text((round(30 * s), y), line, font=f, fill=(245, 247, 249, 220),
               stroke_width=max(1, round(s)), stroke_fill=(0, 0, 0, 120))
        y += round(21 * s)
    return Image.alpha_composite(frame.convert("RGBA"), layer).convert("RGB")


def main():
    take, beats_path, out_prefix = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
    beats = json.loads(beats_path.read_text(encoding="utf-8"))["beats"]
    row = json.loads((take / "run" / "rows.jsonl").read_text(encoding="utf-8").splitlines()[0])
    ep = take / "run" / "episodes" / Path(row["episode_dir"]).name
    trace = json.load(gzip.open(ep / "trace.json.gz")) if (ep / "trace.json.gz").exists() else row["trace"]
    sim0 = float(trace["samples"][0]["sim"])
    replies, fired = row["replies"], row["fired"]
    notes = {"take": str(take), "sim0": sim0, "beats": []}

    # Faithfulness gate: every card is a verbatim excerpt. Split at "…"; each
    # fragment must appear word for word in what was really said or replied.
    def norm(s):
        return " ".join(str(s).replace("−", "-").split())
    for b in beats:
        rep = replies.get(b.get("step") or b.get("answer_from", "")) or {}
        for field, source in (("prompt", rep.get("prompt")), ("answer", rep.get("text"))):
            text = b.get(field)
            if not text:
                continue
            body = text.split(": ", 1)[1] if field == "prompt" and ": " in text else text
            for frag in (f.strip(" .,;") for f in norm(body).split("…")):
                if frag and frag not in norm(source or ""):
                    raise SystemExit(f"Not faithful ({b.get('step') or b.get('at')}, {field}): {frag!r} is not in {source!r}")

    for cam in ("wide", "tall"):
        files = sorted((take / "frames" / cam).glob("*.jpg"))
        stamps = [int(f.stem) / 1000.0 - sim0 for f in files]       # episode seconds

        def frame_at(t):
            i = max(0, bisect_right(stamps, t) - 1)
            return files[i], stamps[i]

        tmp = Path(tempfile.mkdtemp(prefix=f"wshift_{cam}_"))
        n = 0
        for bi, b in enumerate(beats):
            if "at" in b:
                # A beat with no operator message: the robot acting on its own
                # (e.g. a scheduled job). Its answer, if any, is a reply from a
                # later step (`answer_from`), shown at the end of the clip.
                start, end = float(b["at"][0]), float(b["at"][1])
                rep = replies.get(b.get("answer_from", "")) or {}
                t_reply = end if b.get("answer") is not None else None
            else:
                t_fire = fired[b["step"]]["t"]
                rep = replies.get(b["step"]) or {}
                t_reply = rep.get("t_done")
                start = t_fire + float(b.get("from", -0.5))
                to = b.get("to", 6.0)
                end = (t_reply + float(to.split("+")[1])) if isinstance(to, str) and to.startswith("reply+") else t_fire + float(to)
            speed = float(b.get("speed", 1.0))
            out_len = (end - start) / speed
            answer_at = None if (b.get("answer") is None or t_reply is None) else min(out_len, (t_reply - start) / speed)
            # Reading holds: keep every card up for its full time, freezing the last frame if needed.
            need = max(PROMPT_S, (answer_at + ANSWER_S) if answer_at is not None else 0.0)
            freeze = max(0.0, need - out_len)
            total = out_len + freeze
            if cam == "wide":
                notes["beats"].append({"step": b.get("step") or f"at {b['at']}", "sim_window_s": [round(start, 2), round(end, 2)],
                                       "speed": speed, "freeze_s": round(freeze, 2),
                                       "reply_full": rep.get("text"), "via": (rep.get("raw") or {}).get("via")})
            for k in range(round(total * FPS)):
                ot = k / FPS
                t = start + min(ot, out_len) * speed
                path, _ = frame_at(t)
                img = Image.open(path).convert("RGB")
                if b.get("labels", True):
                    img = labels(img, cam)
                prompt = b.get("prompt") if ot < PROMPT_S and (answer_at is None or ot < answer_at) else None
                answer = b["answer"] if answer_at is not None and answer_at <= ot < answer_at + ANSWER_S else None
                img = img.resize(EXPORT[cam], Image.LANCZOS)
                img = overlay(img, prompt=prompt, answer=answer) if cam == "wide" else tall_overlay(img, prompt=prompt, answer=answer)
                if b.get("chip"):
                    img = chip(img, b["chip"])
                if bi == len(beats) - 1 and ot >= total - 3.5:
                    img = footnote(img)
                img.save(tmp / f"{n:06d}.png")
                n += 1
        out = f"{out_prefix}_{cam}.mp4"
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(tmp / "%06d.png"),
                        "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-an", out], check=True)
        shutil.rmtree(tmp, ignore_errors=True)
        print(f"{out}: {n / FPS:.1f} s", flush=True)
    Path(f"{out_prefix}_notes.json").write_text(json.dumps(notes, indent=1, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
