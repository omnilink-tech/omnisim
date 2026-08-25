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

"""Cross-cadence blend adapter for skill references.

BATON blends cyclic specialists element-wise on their reference tables:
``ref_eff = (1-u)*ref_a + u*ref_b``. That requires the two references to share a
**bin count (nb) AND cadence** -- which is exactly why a *sequence* skill (turn ~nb225,
climb ~nb1561, its own root-driven cadence) cannot blend with the fast *cyclic* family
(walk/carry/stand, nb64) and must instead be swapped in solo (``_wr_turn_swap``).

This module supplies the missing half of a true cross-cadence blend: a **phase-faithful
resampler** that maps any ghost's per-bin channels onto a target nb, so a sequence ref
can be re-expressed at the cyclic nb. Resampling fixes the *cadence* mismatch; the
remaining requirement is an *observation-family* match (same obs dim: a 120-dim cyclic
net cannot consume a 153-dim ``REF_OBS_WB`` reference). :func:`blend_report` states
precisely which of the two conditions a pair meets and what is needed to close the gap
(the honest answer for turn today is: retrain the turner in the 120-dim leg-REF family,
then only cadence differs and this resampler closes it).

Pure python (linear interpolation over phase); no numpy / no engine import.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ghost_lut import CHANNELS, GhostLut

# per-bin channels the resampler knows how to interpolate (row lists of floats)
_PERBIN = tuple(c.name for c in CHANNELS)
# companion (non-per-bin) keys copied through unchanged
_COMPANION = ("wb_joints", "kp_bodies", "lp_bodies", "lp_frame")


def _lerp_row(a: Any, b: Any, t: float) -> Any:
    if isinstance(a, (list, tuple)):
        return [ (1.0 - t) * float(x) + t * float(y) for x, y in zip(a, b) ]
    return (1.0 - t) * float(a) + t * float(b)


def resample_channel(rows: list, target_nb: int, seq: bool) -> list:
    """Resample a per-bin channel (nb_src rows) to ``target_nb`` rows, phase-faithful.

    cyclic: phase in [0,1) maps to a wrapped source index (nb_src). sequence: phase in
    [0,1] maps to a clamped source index (nb_src-1) so the endpoints are preserved.
    """
    n_src = len(rows)
    if n_src == 0 or target_nb <= 0:
        return []
    out: list = []
    for j in range(target_nb):
        if seq:
            f = (j / (target_nb - 1)) * (n_src - 1) if target_nb > 1 else 0.0
            i0 = int(f)
            i1 = min(i0 + 1, n_src - 1)
        else:
            f = (j / target_nb) * n_src
            i0 = int(f) % n_src
            i1 = (i0 + 1) % n_src
        out.append(_lerp_row(rows[i0], rows[i1], f - int(f)))
    return out


def resample_lut(lut: dict[str, Any], target_nb: int) -> dict[str, Any]:
    """Return a copy of ``lut`` with every per-bin channel resampled to ``target_nb``.

    Scalar metadata and companion lists are carried through; ``nb`` becomes target_nb.
    The motion duration (``cycle_s``) is unchanged -- only the sampling density changes,
    so ``freq`` is recomputed from cycle_s when present.
    """
    seq = bool(lut.get("seq", False))
    out: dict[str, Any] = {}
    for k, v in lut.items():
        if k in _PERBIN and isinstance(v, list):
            out[k] = resample_channel(v, target_nb, seq)
        else:
            out[k] = v
    out["nb"] = target_nb
    out["resampled_from"] = int(lut.get("nb", 0))
    if lut.get("cycle_s"):
        out["freq"] = target_nb / float(lut["cycle_s"]) if float(lut["cycle_s"]) else lut.get("freq")
    return out


def blend_report(a: GhostLut, b: GhostLut, obs_dim_a: int | None = None,
                 obs_dim_b: int | None = None) -> dict[str, Any]:
    """Assess whether two ghosts can element-wise blend and what (if anything) is needed."""
    cadence_ok = (a.nb == b.nb)
    cadence_fix = None if cadence_ok else f"resample one to nb={min(a.nb, b.nb)} (adapter.resample_lut)"
    obs_ok = (obs_dim_a is None or obs_dim_b is None or obs_dim_a == obs_dim_b)
    obs_fix = None if obs_ok else (
        f"obs family differs ({obs_dim_a} vs {obs_dim_b}); retrain the wider-obs skill in the "
        f"narrower family so both consume the same reference block"
    )
    return {
        "blendable": cadence_ok and obs_ok,
        "cadence_match": cadence_ok,
        "obs_match": obs_ok,
        "needs": [x for x in (cadence_fix, obs_fix) if x],
    }


def _cli(argv: list[str]) -> int:
    """`adapter.py <lut.json> --to-nb N [--out path]` -- resample one lut."""
    import argparse
    ap = argparse.ArgumentParser(description="Resample a ghost lut to a target nb.")
    ap.add_argument("lut")
    ap.add_argument("--to-nb", type=int, required=True)
    ap.add_argument("--out")
    args = ap.parse_args(argv)
    src = GhostLut.load(args.lut)
    out = resample_lut(src.raw, args.to_nb)
    print(f"resampled {Path(args.lut).name}: nb {src.nb} -> {args.to_nb} "
          f"(class={src.motion_class}, channels={src.channels_present})")
    if args.out:
        Path(args.out).write_text(json.dumps(out), encoding="utf-8")
        chk = GhostLut(Path(args.out), out)
        issues = [i for i in chk.validate() if i.level == "error"]
        print(f"wrote {args.out}  ({len(issues)} schema errors)")
    return 0


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    raise SystemExit(_cli(sys.argv[1:]))
