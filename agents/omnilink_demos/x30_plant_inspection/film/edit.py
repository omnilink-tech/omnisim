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

"""Edit a recorded take into the minimal OmniLink demo format.

    python agents/omnilink_demos/x30_plant_inspection/film/edit.py <take_dir> <out.mp4> [last_prompt]

Overlay style: scripts/cinema/styles/omnilink_minimal.py (OMNILINK_DEMOS.md).
Needs Pillow and ffmpeg on PATH.

Native frames only. Edits, all written to edit_notes.json:
  * idle stretches (the robot standing while the model answers) longer than
    IDLE_KEEP_S are cut down to IDLE_KEEP_S -- time compression;
  * a prompt card is shown for PROMPT_S of OUTPUT time from the frame the
    prompt was sent; an answer card (questions / clarifications only) for
    ANSWER_S after the reply, freezing the last frame if the take has no
    footage left for the reading hold.
Nothing is invented: every output frame is a recorded native frame.
"""
import json
import math
import os
import subprocess
import sys

from PIL import Image

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                "..", "..", "..", "..", "scripts", "cinema", "styles")))
from omnilink_minimal import overlay  # noqa: E402

TAKE = sys.argv[1]
OUT = sys.argv[2]
FPS = 25
IDLE_KEEP_S = 1.0
PROMPT_S = 3.0
ANSWER_S = 4.5
MOVE_V = 0.03          # m/s counts as moving
MOVE_W = 0.06          # rad/s counts as turning

frames = [json.loads(l) for l in open(os.path.join(TAKE, "frames", "index.jsonl"))]
events = [json.loads(l) for l in open(os.path.join(TAKE, "events.jsonl"))]
MAXI = int(sys.argv[3]) if len(sys.argv) > 3 else 10 ** 6
cut_sim = min([e["sim"] for e in events if e["event"] == "dispatch" and e["i"] > MAXI] or [1e18])
frames = [f for f in frames if f["sim_time_ms"] / 1000.0 < cut_sim - 0.5]
events = [e for e in events if e.get("i", 0) <= MAXI]
dispatch = [e for e in events if e["event"] == "dispatch"]
# Answer cards: the prompts take.py flagged (show_answer), else these two.
ANSWER_FOR = ({e["text"] for e in dispatch if e.get("show_answer")}
              or {"Go to the tank.", "Where are you?"})
replies = {e["i"]: e for e in events if e["event"] == "reply"}

# moving mask from the recorded poses (sim time, not wall)
mov = []
for k, f in enumerate(frames):
    a = frames[max(0, k - 3)]
    b = frames[min(len(frames) - 1, k + 3)]
    dt = (b["sim_time_ms"] - a["sim_time_ms"]) / 1000.0 or 1.0
    pa, pb = a["poses"]["X30"], b["poses"]["X30"]
    v = math.hypot(pb["x"] - pa["x"], pb["y"] - pa["y"]) / dt
    dyaw = (pb["yaw"] - pa["yaw"] + math.pi) % (2 * math.pi) - math.pi
    mov.append(v > MOVE_V or abs(dyaw) / dt > MOVE_W)


def idx_at(sim_s):
    t = sim_s * 1000.0
    for k, f in enumerate(frames):
        if f["sim_time_ms"] >= t:
            return k
    return len(frames) - 1


# Timeline marks in frame indices. The bridge's sim_time and the capture
# supervisor's clock are the same engine clock.
marks = []
for d in dispatch:
    r = replies.get(d["i"])
    marks.append({"i": d["i"], "text": d["text"], "k_send": idx_at(d["sim"]),
                  "k_reply": idx_at(r["sim"]) if r else None,
                  "answer": (r["reply"].get("response") if r else None)})

# Keep-mask: everything, then shrink idle runs.
keep = [True] * len(frames)
keep_n = int(IDLE_KEEP_S * FPS)
k = 0
cut_total = 0
while k < len(frames):
    if mov[k]:
        k += 1
        continue
    j = k
    while j < len(frames) and not mov[j]:
        j += 1
    run = j - k
    if run > keep_n:
        half = keep_n // 2
        for q in range(k + half, j - (keep_n - half)):
            keep[q] = False
        cut_total += run - keep_n
    k = j
# never cut the send frame of a prompt
for m in marks:
    keep[m["k_send"]] = True

# Build output list of (frame_index, prompt, answer)
out = []
card = None                     # (kind, text, frames_left)
by_send = {}
for m in marks:
    by_send.setdefault(m["k_send"], []).append(m)
by_reply = {}
for m in marks:
    if m["k_reply"] is not None and m["text"] in ANSWER_FOR and m["answer"]:
        by_reply.setdefault(m["k_reply"], []).append(m)
pending_answers = []
for k, f in enumerate(frames):
    for m in by_send.get(k, []):
        card = ("prompt", m["text"], int(PROMPT_S * FPS))
    for m in by_reply.get(k, []):
        pending_answers.append(m)
    if card is None and pending_answers:
        m = pending_answers.pop(0)
        card = ("answer", m["answer"], int(ANSWER_S * FPS))
    if not keep[k] and card is None:
        continue
    out.append((k, card))
    if card is not None:
        card = (card[0], card[1], card[2] - 1) if card[2] > 1 else None
        if card is None and pending_answers:
            m = pending_answers.pop(0)
            card = ("answer", m["answer"], int(ANSWER_S * FPS))
# flush a trailing card by freezing the last frame
while card is not None:
    out.append((out[-1][0], card))
    card = (card[0], card[1], card[2] - 1) if card[2] > 1 else None

# Long uninterrupted walks (no card on screen) play at FAST_X: every
# FAST_X-th frame kept once a run passes FAST_AFTER_S. Noted, never hidden.
FAST_AFTER_S, FAST_X = 12.0, 2
fast_dropped = 0
res, run = [], []
def _flush():
    global fast_dropped
    lim = int(FAST_AFTER_S * FPS)
    if len(run) > lim:
        keep_run = run[:lim // 2] + run[lim // 2:len(run) - lim // 2][::FAST_X] + run[len(run) - lim // 2:]
        fast_dropped += len(run) - len(keep_run)
        res.extend(keep_run)
    else:
        res.extend(run)
    run.clear()
for item in out:
    if item[1] is None and mov[item[0]]:
        run.append(item)
    else:
        _flush(); res.append(item)
_flush()
out = res

os.makedirs(os.path.join(TAKE, "edit"), exist_ok=True)
for n, (k, card) in enumerate(out):
    img = Image.open(os.path.join(TAKE, "frames", frames[k]["file"])).convert("RGB")
    if card is None:
        img = overlay(img)
    elif card[0] == "prompt":
        img = overlay(img, prompt=card[1])
    else:
        img = overlay(img, answer=card[1], robot="X30")
    img.save(os.path.join(TAKE, "edit", "%06d.png" % n))

notes = {"fps": FPS, "source_frames": len(frames), "output_frames": len(out),
         "idle_frames_cut": cut_total, "idle_keep_s": IDLE_KEEP_S,
         "prompt_card_s": PROMPT_S, "answer_card_s": ANSWER_S,
         "frozen_frames": sum(1 for a, b in zip(out, out[1:]) if a[0] == b[0]),
         "fast_frames_dropped": fast_dropped, "fast_after_s": FAST_AFTER_S, "fast_x": FAST_X,
         "marks": marks}
json.dump(notes, open(os.path.join(TAKE, "edit_notes.json"), "w"), indent=1)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
                "-i", os.path.join(TAKE, "edit", "%06d.png"), "-an", "-c:v", "libx264",
                "-pix_fmt", "yuv420p", "-crf", "18", "-movflags", "+faststart", OUT], check=True)
print(json.dumps({k: v for k, v in notes.items() if k != "marks"}))
