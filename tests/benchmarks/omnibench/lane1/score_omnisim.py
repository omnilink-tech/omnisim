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

"""score_omnisim.py — adapt OmniSim lane-1 recordings to the SPEC scorer.

run_omnisim.py records bodies under their world DEF names (BALL, SLIDER,
LINK1, CHAIN, BOX0.., BRICK) and adds joint_<DEF> extra keys; score.py expects
the engine-neutral body names from common/scenes.py (ball, box15.., link1..,
box0.., box) and exactly the pos/vel/quat/omega field set. T2 additionally
runs as 7 per-angle worlds (one SLIDER each) while the scorer wants ONE
recording holding box15..box35.

This module bridges the two, changing NOTHING numerically:
  * renames bodies per the DEF->SPEC map,
  * drops joint_* extras,
  * merges T2's 7 per-angle recordings (truncated to the common step count),
  * writes the adapted recording next to the first input as *.spec.npz,
  * scores it with score.py.

CLI:
    python score_omnisim.py <recordings_dir> --test T2 --dt-ms 4 --backend newton
API:
    metrics, notes, extras, adapted = score_run("T2", 4, "newton", rec_dir)
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import gen_worlds  # noqa: E402
import score as scorer  # noqa: E402
from common import scenes  # noqa: E402
from common.recording import FIELDS  # noqa: E402

# DEF name (recorder key) -> SPEC body name (scenes.BODIES), per test.
DEF_TO_SPEC = {
    "T1": {"BALL": "ball"},
    "T3": {"ROLLER": "ball"},
    "T4": {"LINK1": "link1", "LINK2": "link2", "LINK3": "link3"},
    # T5's free-floating chain: the CHAIN Robot node IS link1.
    "T5": {"CHAIN": "link1", "LINK2": "link2", "LINK3": "link3"},
    "T6": {"BOX%d" % i: "box%d" % i for i in range(10)},
    "T7": {"BRICK": "box"},
}
# T2: each per-angle stem's single body SLIDER maps to box<deg>.


def npz_path(rec_dir, stem, dt_ms, backend):
    return Path(rec_dir) / ("%s_dt%d_%s.npz" % (stem, int(dt_ms), backend))


def _load_raw(path):
    with np.load(path) as z:
        return {k: z[k] for k in z.files}


def _extract(raw, def_name, spec_name, n):
    out = {}
    for f in FIELDS:
        key = "%s_%s" % (f, def_name)
        if key not in raw:
            raise ValueError("recording missing key %r" % key)
        out["%s_%s" % (f, spec_name)] = raw[key][:n]
    return out


def adapt(test, dt_ms, backend, rec_dir):
    """Build the SPEC-named recording for one (test, dt, backend).

    Returns (adapted_path, source_paths). Raises FileNotFoundError /
    ValueError when an input recording is missing or malformed — the caller
    records that as a gap.
    """
    test = scenes.normalize_test_id(test)
    rec_dir = Path(rec_dir)
    stems = gen_worlds.TESTS[test]
    sources = [npz_path(rec_dir, s, dt_ms, backend) for s in stems]
    for p in sources:
        if not p.exists():
            raise FileNotFoundError(str(p))

    if test == "T2":
        raws = [_load_raw(p) for p in sources]
        n = min(r["t"].shape[0] for r in raws)
        out = {"t": raws[0]["t"][:n]}
        for stem, raw in zip(stems, raws):
            deg = int(gen_worlds.SCENES[stem]["angle_deg"])
            out.update(_extract(raw, "SLIDER", "box%d" % deg, n))
    else:
        raw = _load_raw(sources[0])
        n = raw["t"].shape[0]
        out = {"t": raw["t"]}
        for def_name, spec_name in DEF_TO_SPEC[test].items():
            out.update(_extract(raw, def_name, spec_name, n))

    adapted = sources[0].with_name(
        "%s_dt%d_%s.spec.npz" % (scenes.TEST_NAMES[test], int(dt_ms), backend))
    np.savez_compressed(adapted, **out)
    return adapted, sources


def score_run(test, dt_ms, backend, rec_dir):
    """Adapt + score one (test, dt, backend). -> (metrics, notes, extras,
    adapted_path)."""
    test = scenes.normalize_test_id(test)
    adapted, _ = adapt(test, dt_ms, backend, rec_dir)
    metrics, notes, extras = scorer.score(test, adapted)
    return metrics, notes, extras, adapted


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recordings_dir")
    ap.add_argument("--test", required=True, help="T1..T7 or SPEC name")
    ap.add_argument("--dt-ms", type=int, required=True)
    ap.add_argument("--backend", required=True, choices=["ode", "newton"])
    args = ap.parse_args(argv)
    metrics, notes, extras, adapted = score_run(
        args.test, args.dt_ms, args.backend, args.recordings_dir)
    print(json.dumps({"metrics": metrics, "notes": notes, "extras": extras,
                      "adapted": str(adapted)}, indent=2))


if __name__ == "__main__":
    main()
