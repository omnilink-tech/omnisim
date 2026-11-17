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

"""Add a quiet, original ambient bed to a finished demo cut.

    python agents/omnilink_demos/x30_plant_inspection/film/music.py <in.mp4> <out.mp4>

The track is synthesised here from sine partials (OmniLink's own, clear to
post): soft pads, a sub-bass root and a sparse pluck over a slow four-chord
loop, low-passed, faded in over 4 s and out over the last 6 s, mixed quiet
(loudness-normalised to about -24 LUFS) so it sits under the footage rather
than on top of it. No sound effects: nothing here pretends to come from the
simulation. The video stream is copied untouched. Refuses to overwrite.
Needs numpy and ffmpeg on PATH.
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import wave

import numpy as np

SR = 44100
F = {"F2": 87.31, "G2": 98.00, "A2": 110.00, "C3": 130.81, "E3": 164.81,
     "F3": 174.61, "G3": 196.00, "A3": 220.00, "B3": 246.94, "C4": 261.63,
     "D4": 293.66, "E4": 329.63, "G4": 392.00, "A4": 440.00, "C5": 523.25}
# A calm loop, 6 s per chord: C  Am  F  G  (pads mid-low, one pluck a second).
CHORDS = [
    {"pad": ["C3", "G3", "E4"], "bass": "C3", "arp": ["G4", "E4", "C5", "E4"]},
    {"pad": ["A2", "E3", "C4"], "bass": "A2", "arp": ["E4", "C4", "A4", "C4"]},
    {"pad": ["F2", "C4", "A3"], "bass": "F2", "arp": ["A4", "C5", "A4", "F3"]},
    {"pad": ["G2", "D4", "B3"], "bass": "G2", "arp": ["D4", "G4", "B3", "D4"]},
]
CHORD_S = 6.0


def env(n, a, d, s, r):
    a, d, r = int(a * SR), int(d * SR), int(r * SR)
    hold = max(0, n - a - d - r)
    e = np.concatenate([np.linspace(0, 1, a, endpoint=False), np.linspace(1, s, d, endpoint=False),
                        np.full(hold, s), np.linspace(s, 0, r)])
    return np.pad(e, (0, max(0, n - len(e))))[:n]


def voice(buf, t0, freq, dur, amp, kind):
    s = int(t0 * SR)
    n = min(int(dur * SR), len(buf) - s)
    if n <= 0:
        return
    t = np.arange(n) / SR
    if kind == "pad":        # slow swell, a little detune for width
        w = sum(h * np.sin(2 * np.pi * freq * k * t * (1 + dt))
                for k, h in ((1, 1.0), (2, 0.25)) for dt in (-0.002, 0.002)) / 2
        e = env(n, 1.8, 0.8, 0.8, 1.8)
    elif kind == "bass":
        w = np.sin(2 * np.pi * freq * t)
        e = env(n, 0.4, 0.5, 0.7, 1.0)
    else:                    # pluck
        w = np.sin(2 * np.pi * freq * t) + 0.2 * np.sin(4 * np.pi * freq * t)
        e = env(n, 0.006, 1.2, 0.0, 0.05)
    buf[s:s + n] += amp * w * e


def synth(duration_s: float) -> np.ndarray:
    buf = np.zeros(int((duration_s + 2) * SR))
    t, i = 0.0, 0
    while t < duration_s + 2:
        ch = CHORDS[i % len(CHORDS)]
        for note in ch["pad"]:
            voice(buf, t, F[note], CHORD_S + 1.5, 0.10, "pad")
        voice(buf, t, F[ch["bass"]] / 2, CHORD_S + 0.5, 0.18, "bass")
        for k in range(int(CHORD_S)):
            if (i + k) % 3 != 2:             # sparse: skip every third beat
                voice(buf, t + k, F[ch["arp"][k % 4]], 1.4, 0.07, "pluck")
        t += CHORD_S
        i += 1
    buf = buf[:int(duration_s * SR)]
    return buf / (np.max(np.abs(buf)) or 1.0) * 0.8


def duration(path: str) -> float:
    out = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration",
                          "-of", "csv=p=0", path], capture_output=True, text=True, check=True)
    return float(out.stdout.strip())


def main() -> int:
    src, dst = sys.argv[1], sys.argv[2]
    if os.path.exists(dst):
        raise SystemExit(f"refusing to overwrite {dst}")
    dur = duration(src)
    pcm = (synth(dur) * 32767).astype(np.int16)
    with tempfile.TemporaryDirectory() as tmp:
        wav = os.path.join(tmp, "bed.wav")
        with wave.open(wav, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(SR)
            w.writeframes(pcm.tobytes())
        af = (f"lowpass=f=6000,highpass=f=35,afade=t=in:st=0:d=4,"
              f"afade=t=out:st={max(0.0, dur - 6):.2f}:d=6,loudnorm=I=-24:TP=-3:LRA=7,"
              f"aformat=channel_layouts=stereo")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", src, "-i", wav,
                        "-filter_complex", f"[1:a]{af}[a]", "-map", "0:v", "-map", "[a]",
                        "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-ar", "48000",
                        "-shortest", "-movflags", "+faststart", dst], check=True)
    print(f"wrote {dst} ({os.path.getsize(dst) // 1024} KB, {dur:.1f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
