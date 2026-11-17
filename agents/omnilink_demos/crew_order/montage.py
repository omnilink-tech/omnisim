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
"""The stability montage: 16 runs at once, fast-forwarded, then the tally.

    python agents/omnilink_demos/crew_order/montage.py <campaign_dir> <out.mp4> [--film draft.mp4 --final final.mp4]

Each tile is one campaign run's own `tile` camera, from one second before
Sam's order to a few seconds after that run's verdict, played at a constant
speed (exactly STEP captured frames per output frame). A tile's mark comes
from that run's result.json: a check only when every part was measured in the
crate AND the robots reported to Sam; anything else shows a cross. The closing
card prints summary.json and nothing else.
"""
import argparse
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
import take  # noqa: E402  (report_time: the same report rule the verdict uses)
FPS = 24
STEP = 2                       # captured frames per output frame (256 ms frames -> 12.3x)
W, H = 1920, 1080
TW, TH = 480, 255             # the grid sits under a 60 px banner strip
TOP = 60
FONT_SYMBOL = "C:/Windows/Fonts/seguisym.ttf"   # Segoe UI has no check or cross glyph
# OmniLink brand (resources/branding/omnilink/BRAND.md): black, cream, mimosa;
# Hogira for display titles, Montserrat for everything else. Hogira is not
# redistributed, so a public checkout falls back to Montserrat Bold.
BRAND = HERE.parents[2] / "resources" / "branding" / "omnilink"
BLACK, CREAM, MIMOSA = (0, 0, 0), (238, 238, 224), (246, 233, 5)
CREAM_MUTE = (176, 176, 164)
FONT = str(BRAND / "fonts" / "Montserrat-Medium.otf")
FONT_BOLD = str(BRAND / "fonts" / "Montserrat-SemiBold.otf")
FONT_DISPLAY = BRAND / "fonts" / "Hogira-Bold.otf"
OK_RGB, BAD_RGB = MIMOSA, (255, 110, 90)


def font(n, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, n)


def display_font(n):
    return ImageFont.truetype(str(FONT_DISPLAY) if FONT_DISPLAY.exists() else str(BRAND / "fonts" / "Montserrat-Bold.ttf"), n)


def brand_image(rel, height):
    im = Image.open(BRAND / rel).convert("RGBA")
    return im.resize((round(im.width * height / im.height), height), Image.LANCZOS)


def load_run(run: Path):
    rows = [json.loads(x) for x in (run / "crew.jsonl").read_text(encoding="utf-8").splitlines()]
    res = json.loads((run / "result.json").read_text(encoding="utf-8"))
    order_t = next(r["sim_s"] for r in rows if r["kind"] == "operator" and "Order" in r["text"])
    # the COMPLETION report, judged as take.judge() judges it -- not the first
    # message to Sam, which can be an early progress note (campaign_sonnet run 7)
    end_t = take.report_time(rows)
    final_t = next((r["sim_s"] for r in reversed(rows) if r["kind"] == "final" and "sim_s" in r), None)
    files = sorted((run / "frames" / "tile").glob("*.jpg"))
    stamps = [int(p.stem) / 1000.0 for p in files]
    i0 = max(0, bisect_right(stamps, order_t - 1.0) - 1)
    verdict_t = end_t if end_t is not None else (final_t or stamps[-1])
    i1 = min(len(files) - 1, bisect_right(stamps, verdict_t) - 1)
    return {"files": files, "stamps": stamps, "i0": i0, "i1": i1, "order_t": order_t, "res": res}


def tile(run, k, name):
    """Output frame k of one run's tile: playing, then holding its last frame with the verdict."""
    i = min(run["i0"] + k * STEP, run["i1"])
    done = run["i0"] + k * STEP >= run["i1"]
    img = None
    for j in range(i, -1, -1):            # a frame cut short by the engine stopping mid-write
        try:
            img = Image.open(run["files"][j]).convert("RGB").resize((TW, 270), Image.LANCZOS).crop((0, 7, TW, 7 + TH))
            break
        except OSError:
            continue
    d = ImageDraw.Draw(img, "RGBA")
    t = run["stamps"][i] - run["order_t"]
    d.rounded_rectangle((8, TH - 34, 150, TH - 8), radius=6, fill=(0, 0, 0, 210))
    d.text((16, TH - 33), f"{name}  {max(0, int(t)) // 60}:{max(0, int(t)) % 60:02d}", font=font(17, True),
           fill=(245, 247, 249, 255))
    if done:
        ok = bool(run["res"].get("ok"))
        col = OK_RGB if ok else BAD_RGB
        d.rectangle((0, 0, TW - 1, TH - 1), outline=(*col, 255), width=4)
        mark = "✓" if ok else "✗"
        label = "packed" if ok else ("not packed" if not run["res"].get("packed") else "no report")
        sym = ImageFont.truetype(FONT_SYMBOL, 22)
        mw, tw = d.textlength(mark, font=sym), d.textlength(label, font=font(22, True))
        x0 = TW - tw - mw - 40
        d.rounded_rectangle((x0, 10, TW - 10, 46), radius=8, fill=(0, 0, 0, 225))
        d.text((x0 + 11, 12), mark, font=sym, fill=(*col, 255))
        d.text((x0 + 17 + mw, 12), label, font=font(22, True), fill=(*col, 255))
    return img, done


def card(summary, seconds, tmp, n0):
    n = n0
    lines = [
        (f"{summary['runs']} independent runs", 72, "display", CREAM),
        (f"{summary['ok']} packed and reported", 56, "display", OK_RGB if summary["ok"] == summary["runs"] else BAD_RGB),
        (f"{summary['runs'] - summary['ok']} failed · {summary['crashed']} crashed · "
         f"{summary['delivery_errors']} message-delivery errors", 34, False, CREAM),
        (f"{summary.get('start_failures_retried', 0)} engine start failure(s) before any robot was briefed, re-run"
         if summary.get("start_failures_retried") else "", 26, False, CREAM_MUTE),
        (f"{summary.get('quota_failures_retried', 0)} run(s) refused by the test account (agent quota or API credit), re-run"
         if summary.get("quota_failures_retried") else "", 26, False, CREAM_MUTE),
        # free-text facts a summary carries (a campaign stopped early, a refusal the crew recovered from)
        *[(note, 26, False, CREAM_MUTE) for note in summary.get("notes", [])],
        (f"Each run: three robots, each its own OmniLink agent ({ {'claude-opus-5-5': 'Claude Opus 5.5', 'claude-sonnet-5-5': 'Claude Sonnet 5.5'}.get(summary.get('model'), summary.get('model', 'Claude')) }), one order from Sam.", 24, False, CREAM_MUTE),
        ("Simulated in OmniSim. A run passes only if all three parts were measured in the crate and Sam got the report.", 24, False, CREAM_MUTE),
    ]
    img = Image.new("RGB", (W, H), BLACK)
    orb = brand_image("orb/orb_512.png", 220)
    img.paste(orb, ((W - orb.width) // 2, 70), orb)
    d = ImageDraw.Draw(img)
    y = 330
    for text, size, bold, col in lines:
        if not text:
            continue
        f = display_font(size) if bold == "display" else font(size, bold)
        d.text(((W - d.textlength(text, font=f)) / 2, y), text, font=f, fill=col)
        y += int(size * 1.6)
        if size == 56:            # a mimosa rule under the verdict
            d.rectangle(((W - 160) // 2, y - 8, (W + 160) // 2, y - 5), fill=MIMOSA)
            y += 14
    lockup = brand_image("png/lockup_horizontal_yellow.png", 64)
    img.paste(lockup, ((W - lockup.width) // 2, H - 110), lockup)
    for _ in range(int(seconds * FPS)):
        img.save(tmp / f"{n:06d}.png")
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign")
    ap.add_argument("out")
    ap.add_argument("--film", default=None, help="the single-run film to put first")
    ap.add_argument("--final", default=None, help="write film + montage concatenated here")
    ap.add_argument("--summary", default=None, help="summary.json to print (default: the campaign's own)")
    a = ap.parse_args()
    camp = Path(a.campaign)
    summary = json.loads(Path(a.summary or camp / "summary.json").read_text(encoding="utf-8"))
    runs = []
    for k in range(1, 17):
        run = camp / f"run_{k:03d}"
        if (run / "result.json").exists() and (run / "frames" / "tile").exists():
            runs.append((f"RUN {k:02d}", load_run(run)))
    tmp = Path(tempfile.mkdtemp(prefix="crew_montage_"))
    longest = max(r["i1"] - r["i0"] for _, r in runs) // STEP + 1
    banner_lockup = brand_image("png/lockup_horizontal_yellow.png", 44)
    n = 0
    for k in range(longest + FPS * 2):
        canvas = Image.new("RGB", (W, H), BLACK)
        for j, (name, run) in enumerate(runs):
            img, _ = tile(run, k, name)
            canvas.paste(img, ((j % 4) * TW, TOP + (j // 4) * TH))
        d = ImageDraw.Draw(canvas, "RGBA")
        banner = f"{len(runs)} of {summary['runs']} runs, side by side · each from its own order · {6.144 * STEP:.0f}× speed"
        f = font(26, True)
        tw = d.textlength(banner, font=f)
        d.text(((W - tw) / 2, 13), banner, font=f, fill=(*CREAM, 255))
        canvas.paste(banner_lockup, (20, 8), banner_lockup)
        d.rectangle((0, TOP - 3, W, TOP - 1), fill=(*MIMOSA, 255))
        canvas.save(tmp / f"{n:06d}.png")
        n += 1
    n = card(summary, 5.0, tmp, n)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(tmp / "%06d.png"),
                    "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-an", a.out], check=True)
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"{a.out}: {n / FPS:.1f} s, {len(runs)} tiles")
    if a.film and a.final:
        lst = Path(a.out).with_suffix(".concat.txt")
        lst.write_text(f"file '{Path(a.film).resolve().as_posix()}'\nfile '{Path(a.out).resolve().as_posix()}'\n", encoding="utf-8")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst),
                        "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-an", a.final], check=True)
        print(f"{a.final}: film + montage")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
