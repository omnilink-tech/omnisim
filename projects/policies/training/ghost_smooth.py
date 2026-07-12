#!/usr/bin/env python3
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

"""ghost_smooth.py -- harmonic-smooth a recorded ghost lut (the fold->smooth stage of the
record pipeline, standalone). A raw EVAL_RECORD single cycle neither closes at the wrap nor
is spectrally clean; truncating each channel to its first H harmonics makes it PERIODIC BY
CONSTRUCTION and removes capture jitter, which is what the validator's phase-wrap and
harmonic-jitter gates check.

Usage: python ghost_smooth.py <lut.json> [--harmonics 8]
Smooths leg_lut / arm_lut / att_lut / wb_lut / elbow_lut / waist_lut in place.
"""
import json
import sys

import numpy as np


def _smooth(tab, H):
    t = np.asarray(tab, np.float32)
    F = np.fft.rfft(t, axis=0)
    F[H + 1:] = 0.0
    return np.fft.irfft(F, n=t.shape[0], axis=0).astype(np.float32)


def main(path, H=8):
    d = json.load(open(path))
    done = []
    for k in ("leg_lut", "arm_lut", "att_lut", "wb_lut", "elbow_lut", "waist_lut"):
        if k in d:
            arr = np.asarray(d[k], np.float32)
            if arr.ndim == 1:
                arr = arr[:, None]
                d[k] = _smooth(arr, H)[:, 0].tolist()
            else:
                d[k] = _smooth(arr, H).tolist()
            done.append(k)
    d["source"] = (d.get("source", "") + f" | harmonic-smoothed H={H}")[:400]
    json.dump(d, open(path, "w"))
    print(f"[smooth] {path}: H={H} on {done}")


if __name__ == "__main__":
    H = 8
    if "--harmonics" in sys.argv:
        H = int(sys.argv[sys.argv.index("--harmonics") + 1])
    main(sys.argv[1], H)
