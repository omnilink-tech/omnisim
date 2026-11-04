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

"""Test parts with an exact formula, so a milled part can be measured against the truth.

Work coordinates: Z0 = stock top, the part's surface at Z <= 0.

    python parts.py mould --out mould.stl          # an STL any CAM can read
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import toolpath3d  # noqa: E402


def mould(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """A 70 mm mould cavity, 10 mm deep, with 6 mm corner radii. Standing in it:

    - a spherical dome, 32 mm across, its top 1 mm below the stock top;
    - a ring of round section (25 mm radius, 3.5 mm tube) around the dome;
    - four bosses with 45-degree chamfers, flat tops 4 mm below the stock top.
    """
    x, y = np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64)
    half, corner, floor = 35.0, 6.0, -10.0
    qx, qy = np.abs(x) - (half - corner), np.abs(y) - (half - corner)
    outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - corner
    z = np.full(np.broadcast(x, y).shape, floor)
    r = np.hypot(x, y)
    # Dome: a sphere cap 16 mm in radius at the floor, rising to Z -1.
    cap_r, cap_top = 16.0, -1.0
    h = cap_top - floor
    R = (cap_r ** 2 + h ** 2) / (2 * h)
    z = np.maximum(z, np.where(r < cap_r, cap_top - R + np.sqrt(np.maximum(R * R - r * r, 0)), floor))
    # Ring: a half torus on the floor.
    ring = 3.5 - np.abs(r - 25.0)
    z = np.maximum(z, np.where(ring > 0, floor + np.sqrt(np.maximum(3.5 ** 2 - (r - 25.0) ** 2, 0)), floor))
    # Bosses: flat 5 mm tops at Z -4, 45-degree chamfer down to the floor.
    for sx in (-1, 1):
        for sy in (-1, 1):
            d = np.hypot(x - 28.0 * sx, y - 28.0 * sy)
            z = np.maximum(z, np.minimum(-4.0, -4.0 - (d - 2.5)))
    return np.where(outside > 0, 0.0, np.maximum(z, floor))


PARTS = {"mould": mould}


def part_solid(name: str, size: float = 90.0, res: float = 0.25, bottom: float = -15.0) -> np.ndarray:
    """The part as a closed mesh: its formula sampled every `res` mm over a `size` mm square."""
    xs = np.arange(-size / 2, size / 2 + res / 2, res)
    X, Y = np.meshgrid(xs, xs)
    return toolpath3d.heightfield_solid(PARTS[name](X, Y), xs[0], xs[0], res, bottom)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("part", choices=sorted(PARTS))
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--size", type=float, default=90.0, help="square sampled, mm")
    ap.add_argument("--res", type=float, default=0.25, help="mesh spacing, mm")
    args = ap.parse_args(argv)
    tris = part_solid(args.part, args.size, args.res)
    toolpath3d.write_stl(args.out, tris)
    print(f"wrote {args.out}: {len(tris)} triangles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
