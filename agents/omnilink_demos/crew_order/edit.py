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
"""Cut the crew-order film from a take: the floor on the left, the crew's
channel on the right, every line verbatim from <take>/crew.jsonl.

    python agents/omnilink_demos/crew_order/edit.py <take_dir> <out_prefix>

Time runs on the simulator clock (frames are named by sim ms). Every stretch in
which anything on screen moves plays at ONE speed -- exactly one captured frame
per output frame (64 ms frames at 24 fps = 1.536x), so the picture always advances
by the same step and a robot never
looks fast in one shot and slow in the next. Motion is measured from the frames
themselves (any camera). Still stretches -- the robots idle while their agents
think -- are skipped through at IDLE_SPEED, and slowed only to give a new chat
line its reading time. A corner chip always says which of the two is playing. The camera follows the work: the
wide shot while the tug is moving or away from both pads, otherwise the close
shot of the pad it is parked at. Shown: Sam's order, every robot-to-robot
message and the report to Sam. Left out (they are in the log): the briefings
and the delivery replies. A line can be abridged with "…" only through
<take>/abridge.json ({full text: short}); every fragment of the short form must
be verbatim in the full text, or the edit refuses.
"""
import argparse
import json
import math
import shutil
import subprocess
import sys
import tempfile
from bisect import bisect_right
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import world  # noqa: E402

FPS = 24
FONT = "C:/Windows/Fonts/segoeui.ttf"
FONT_BOLD = "C:/Windows/Fonts/segoeuib.ttf"
W, H = 1920, 1080
VIEW_W = 1290                       # picture area; the crew channel fills the rest
COLOURS = {"Sam": (235, 238, 242), "Picker": (255, 170, 64), "Tug": (111, 231, 208), "Packer": (150, 170, 255)}
IDLE_SPEED = 20.0                   # sim s per output s while nothing moves and nothing is being read
MOTION_DIFF = 0.04                  # mean grey-level change between frames that counts as motion
WORDS_PER_S = 4.0                   # reading pace for a new line
TITLE = "One order. Three robots. They work out the rest between themselves."
MODEL_NAMES = {"claude-opus-5-5": "Claude Opus 5.5", "claude-sonnet-5-5": "Claude Sonnet 5.5"}
FOOTNOTE = ("Simulated in OmniSim · each robot runs its own OmniLink agent ({model}) · "
            "after Sam's one order, every message is robot to robot, verbatim")


def font(n, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, round(n))


def wrap(d, text, f, width):
    lines, line = [], ""
    for word in text.split():
        cand = (line + " " + word).strip()
        if line and d.textlength(cand, font=f) > width:
            lines.append(line)
            line = word
        else:
            line = cand
    return lines + ([line] if line else [])


def chat_lines(take):
    abridge = {}
    if (take / "abridge.json").exists():
        abridge = json.loads((take / "abridge.json").read_text(encoding="utf-8"))
    out = []
    for line in (take / "crew.jsonl").read_text(encoding="utf-8").splitlines():
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r["kind"] == "operator" and r["text"].startswith("Sam: Order"):
            text = r["text"].split(": ", 1)[1]
        elif r["kind"] == "message":
            # A crew message is its REQUEST (what the crewmate's gate judged) and
            # its DETAILS (facts beside it); the card shows both, request first.
            request, details = r["text"], r.get("details") or ""
            text = details if request.strip().lower() in ("", "none") else (
                f"{request} {details}".strip())
        else:
            continue
        short = abridge.get(text, text)
        for frag in (x.strip(" .,;") for x in short.split("…")):
            if frag and frag not in text:
                raise SystemExit(f"Not faithful: {frag!r} is not in {text!r}")
        out.append({"t": float(r["sim_s"]), "from": r["from"], "to": r["to"], "text": short,
                    "read_s": 1.5 + len(short.split()) / WORDS_PER_S})
    return out


def tug_track(take):
    pts = []
    for line in (take / "tug_track.jsonl").read_text(encoding="utf-8").splitlines():
        r = json.loads(line)
        if r.get("sim_s") is not None and r.get("x") is not None:
            pts.append((float(r["sim_s"]), float(r["x"]), float(r["y"])))
    return sorted(pts)


def tug_at(track, t):
    if not track or t <= track[0][0]:
        return world.TUG_START[0], world.TUG_START[1], 0.0
    i = bisect_right([p[0] for p in track], t)
    if i >= len(track):
        return track[-1][1], track[-1][2], 0.0
    (t0, x0, y0), (t1, x1, y1) = track[i - 1], track[i]
    a = (t - t0) / max(1e-6, t1 - t0)
    return x0 + a * (x1 - x0), y0 + a * (y1 - y0), math.hypot(x1 - x0, y1 - y0) / max(1e-6, t1 - t0)


def camera_for(track, t):
    x, _, speed = tug_at(track, t)
    if speed > 0.05:
        return "wide"
    for cam, place in (("picker", "pick cell"), ("packer", "shipping")):
        if abs(x - world.PLACES[place][0]) < 0.4:
            return cam
    return "wide"


def tags(img, cam, track, t, speaking):
    d = ImageDraw.Draw(img, "RGBA")
    tx, ty, _ = tug_at(track, t)
    _, _, _, cw, ch = world.CAMERAS[cam]
    sx, sy = VIEW_W / cw, H / ch
    for name, p in (("Picker", (*world.PICKER, 1.12)), ("Packer", (*world.PACKER, 1.12)), ("Tug", (tx, ty, 0.5))):
        u, v = world.project(cam, p)
        u, v = u * sx, v * sy
        if not (40 < u < VIEW_W - 40 and 40 < v < H - 40):
            continue
        f = font(21 if cam != "wide" else 17, True)
        label = name.upper()
        tw = d.textlength(label, font=f)
        hot = name in speaking
        d.rounded_rectangle((u - tw / 2 - 11, v - 34, u + tw / 2 + 11, v - 2), radius=8,
                            fill=(*COLOURS[name], 235) if hot else (12, 20, 29, 210))
        d.text((u - tw / 2, v - 32), label, font=f, fill=(12, 20, 29, 255) if hot else (*COLOURS[name], 255))
    return img


def panel(img, lines, now, fresh_ids):
    d = ImageDraw.Draw(img, "RGBA")
    d.rectangle((VIEW_W, 0, W, H), fill=(12, 20, 29, 255))
    d.text((VIEW_W + 34, 28), "OmniLink", font=font(30, True), fill=(245, 247, 249, 255))
    d.text((VIEW_W + 34, 70), "crew channel", font=font(20), fill=(150, 160, 172, 255))
    body_w = W - VIEW_W - 68
    blocks = []
    for m in (m for m in lines if m["t"] <= now):
        tl = wrap(d, m["text"], font(21), body_w)
        blocks.append((m, tl, 32 + 29 * len(tl) + 24))
    y = H - 36
    for m, tl, h in reversed(blocks):          # newest at the bottom; older lines scroll up and off
        y -= h
        if y < 112:
            break
        fresh = m["i"] in fresh_ids
        if fresh:
            d.rounded_rectangle((VIEW_W + 18, y - 8, W - 18, y + h - 16), radius=12, fill=(28, 40, 54, 255))
        head = f"{m['from'].upper()}  →  {m['to'].upper()}"
        d.text((VIEW_W + 34, y), head, font=font(17, True), fill=(*COLOURS.get(m["from"], (230, 230, 230)), 255))
        yy = y + 29
        for ln in tl:
            d.text((VIEW_W + 34, yy), ln, font=font(21), fill=(246, 248, 250, 255 if fresh else 175))
            yy += 29
    return img


def chip(img, text, x, y):
    d = ImageDraw.Draw(img, "RGBA")
    f = font(18, True)
    tw = d.textlength(text, font=f)
    d.rounded_rectangle((x - 12, y - 6, x + tw + 12, y + 28), radius=8, fill=(12, 20, 29, 215))
    d.text((x, y - 1), text, font=f, fill=(245, 247, 249, 255))


def caption(img, text, size, y_from_bottom):
    d = ImageDraw.Draw(img, "RGBA")
    f = font(size)
    lines = wrap(d, text, f, VIEW_W - 70)
    y = H - y_from_bottom - len(lines) * round(size * 1.35)
    for ln in lines:
        d.text((34, y), ln, font=f, fill=(245, 247, 249, 240), stroke_width=2, stroke_fill=(0, 0, 0, 150))
        y += round(size * 1.35)


def open_frame(files, i):
    """The frame at i, or the nearest earlier readable one (the last frame of
    a take can be cut short when the engine is stopped mid-write)."""
    for j in range(i, -1, -1):
        try:
            return Image.open(files[j]).convert("RGB")
        except OSError:
            continue
    raise OSError("no readable frame")


def motion_mask(frames):
    """Per wide-frame time: did anything move, in any camera, since the previous frame?"""
    import numpy as np
    stamps = frames["wide"][1]
    moving = np.zeros(len(stamps), dtype=bool)
    for cam, (files, st) in frames.items():
        prev = None
        for i, p in enumerate(files):
            try:
                g = np.asarray(Image.open(p).convert("L").resize((128, 108), Image.BILINEAR), dtype=np.float32)
            except OSError:      # the frame being written when the engine stopped
                continue
            if prev is not None and float(np.abs(g - prev).mean()) > MOTION_DIFF:
                j = min(len(stamps) - 1, bisect_right(stamps, st[i]) - 1)
                moving[max(0, j - 1):j + 1] = True
            prev = g
    return stamps, moving


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("take")
    ap.add_argument("out_prefix")
    a = ap.parse_args()
    take = Path(a.take)
    lines = chat_lines(take)
    meta = json.loads((take / "meta.json").read_text(encoding="utf-8")) if (take / "meta.json").exists() else {}
    model_name = MODEL_NAMES.get(meta.get("model", "claude-opus-5-5"), meta.get("model", "Claude"))
    for i, m in enumerate(lines):
        m["i"] = i
    track = tug_track(take)
    frames = {}
    for cam in world.CAMERAS:
        files = sorted((take / "frames" / cam).glob("*.jpg"))
        frames[cam] = (files, [int(p.stem) / 1000.0 for p in files])
    stamps, moving = motion_mask(frames)
    # Motion advances exactly one captured frame per output frame: an even step.
    # (At 2x, 83 ms per output frame against 64 ms frames stepped 1,1,2,... and judders.)
    period = sorted(b - a for a, b in zip(stamps, stamps[1:]))[len(stamps) // 2]
    motion_speed = period * FPS

    def moves(ta, tb):
        lo, hi = bisect_right(stamps, ta) - 1, bisect_right(stamps, tb)
        return bool(moving[max(0, lo):max(lo + 1, hi)].any())

    t0 = max(stamps[0], lines[0]["t"] - 2.0)
    t_end = min(stamps[-1], lines[-1]["t"] + 2.0)
    tmp = Path(tempfile.mkdtemp(prefix="crew_edit_"))
    notes = {"take": str(take), "sim_window_s": [round(t0, 2), round(t_end, 2)], "motion_speed": round(motion_speed, 3),
             "idle_speed": IDLE_SPEED, "cuts": [], "lines": lines}
    t, n, last_cam, appear, tail = t0, 0, None, {}, 0
    while t <= stamps[-1]:
        now_out = n / FPS
        for m in lines:
            if m["t"] <= t and m["i"] not in appear:
                appear[m["i"]] = now_out
        fresh = {i for i, a0 in appear.items() if now_out - a0 < lines[i]["read_s"]}
        if t > t_end and not fresh:
            tail += 1
            if tail > FPS * 3:
                break
        step_motion = moves(t, t + period)
        speed = motion_speed if step_motion else (1.0 if fresh else IDLE_SPEED)
        if not step_motion and not fresh and moves(t, t + IDLE_SPEED / FPS):
            step_motion = True                   # never jump past the start of a movement
            speed = motion_speed
        cam = camera_for(track, t)
        if cam != last_cam:
            notes["cuts"].append({"sim_s": round(t, 2), "camera": cam})
            last_cam = cam
        files, st = frames[cam]
        img = open_frame(files, max(0, bisect_right(st, t) - 1)).resize((VIEW_W, H), Image.LANCZOS)
        canvas = Image.new("RGB", (W, H))
        canvas.paste(img, (0, 0))
        speaking = {lines[i]["from"] for i in fresh} | {lines[i]["to"] for i in fresh}
        canvas = tags(canvas, cam, track, t, speaking)
        canvas = panel(canvas, lines, t, fresh)
        chip(canvas, f"{motion_speed:.1f}× speed" if step_motion else
             ("robots idle, agents thinking · skipping ahead" if speed == IDLE_SPEED else "robots idle"), 34, H - 58)
        if n < FPS * 5:
            caption(canvas, TITLE, 30, 110)
        if t > lines[-1]["t"] + 1.0:          # after the last line only
            caption(canvas, FOOTNOTE.format(model=model_name), 18, 100)
        canvas.save(tmp / f"{n:06d}.png")
        n += 1
        if step_motion:
            i = bisect_right(stamps, t + 1e-6) - 1
            t = stamps[min(i + 1, len(stamps) - 1)] if i + 1 < len(stamps) else t + period
        else:
            t += speed / FPS
    out = f"{a.out_prefix}_wide.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(tmp / "%06d.png"),
                    "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-an", out], check=True)
    shutil.rmtree(tmp, ignore_errors=True)
    Path(f"{a.out_prefix}_notes.json").write_text(json.dumps(notes, indent=1, ensure_ascii=False), encoding="utf-8")
    print(f"{out}: {n / FPS:.1f} s, sim {t0:.1f}-{t:.1f}, {len(notes['cuts'])} cuts, "
          f"{int(moving.sum())}/{len(moving)} frames moving")


if __name__ == "__main__":
    main()
