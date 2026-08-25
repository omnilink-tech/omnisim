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

# D1.5 (WREN deleted at D1.4, commit 976b9449d): the WREN renderer no longer exists, so
# every WREN arm this tool could drive is RETIRED -- the engine warns about and ignores
# the retired selectors, and a "WREN arm" run renders wgpu. The frozen WREN reference
# images live in tests/rendering/wren_reference/ (captured pre-deletion). The tool itself
# is kept: its wgpu arms and its A/B harness remain useful.
"""sensor_oracle.py -- the WREN-vs-wgpu SENSOR parity oracle (WREN-retirement W3).

WHAT THIS TOOL ESTABLISHES
--------------------------
For one world, it renders the SAME Camera device twice -- once with
`Camera.renderBackend "wren"`, once with `"wgpu"` -- and diffs the bytes the
CONTROLLER receives from `wb_camera_get_image()`. Nothing else about the world
changes: same file, same controller, same scene, same step count; one string
field differs. The output is a measured per-channel comparison plus an explicit
channel-swap test.

WHAT THIS TOOL DOES *NOT* ESTABLISH
-----------------------------------
 * It is not a main-view test. `render_ab.py` / `render_oracle.py` own that.
 * It is not a self-consistency test. `wgpu_sensor_regression.py` checks the
   wgpu arm against HAND-COMPUTED expectations on synthetic probe worlds, and
   `wgpu_probe_golden.py` checks wgpu against STORED wgpu. Both can be fully
   green while wgpu and WREN disagree, because neither ever renders the WREN
   arm. That gap is the reason this file exists.
 * A MATCH says "these two arms produced the same controller-visible bytes on
   THIS world at THIS step count". It says nothing about worlds not run, about
   any material / light / post-FX feature the world does not exercise, or about
   temporal behaviour (the controller reads a single frame after 3 steps).
 * It covers the Camera device ONLY. RangeFinder and Lidar are deliberately out
   of scope: their backend selector is a process-wide env var
   (`OMNISIM_RANGEFINDER_WGPU` / `OMNISIM_LIDAR_WGPU`), not a per-node field,
   and their controller-visible payload is a float range buffer, not BGRA -- a
   different comparator, not a flag on this one. Do not extend this tool by
   pretending a range buffer is an image.
 * It cannot by itself tell "the two backends agree" from "the two backends are
   both WREN". A build without wgpu-native, or an ambient `OMNISIM_FORCE_WREN` /
   `OMNISIM_LEGACY`, silently resolves the wgpu arm to WREN
   (`OmRenderBackendRegistry::resolve`, src/omnisim/render/OmRenderBackend.cpp)
   and a vacuous MATCH is the result. So the tool GATES on positive engine
   evidence -- see EVIDENCE below -- and reports WGPU_NOT_ENGAGED rather than
   MATCH when that evidence is absent.

EVIDENCE (why a verdict here is worth anything)
----------------------------------------------
 * wgpu arm: the engine must log `[OmCamera] '<cam>' rendered through wgpu`
   (src/omnisim/nodes/OmCamera.cpp -- a deliberate one-shot marker added so
   harnesses can tell the wgpu path fired from a silent WREN fall-through).
   Zero hits => verdict WGPU_NOT_ENGAGED, exit non-zero.
 * wren arm: that marker must be ABSENT. This is NEGATIVE evidence and is named
   as such in the JSON (`wren_arm_evidence: "absence_of_wgpu_marker"`) -- WREN
   emits no positive per-camera marker, so the WREN arm is established by the
   world's declared `renderBackend "wren"` plus the absence of the wgpu line,
   not by a WREN-side confirmation. If a WREN-side marker is ever added,
   tighten this.
 * Both arms report their engine ERROR / WARNING counts; they are in the JSON.

CHANNEL ORDER -- the thing a single mean would hide
---------------------------------------------------
The controller API is BGRA: `wb_camera_image_get_red(image,w,x,y)` is
`image[i+2]` and `_get_blue` is `image[i]`
(include/controller/c/omnisim/camera.h:87-89). The `camera_wgpu_smoke`
controller writes its PPM by applying exactly that convention
(rgb[0]=raw[i+2], rgb[1]=raw[i+1], rgb[2]=raw[i]), identically on both arms.
So if one backend hands the controller RGBA where the other hands BGRA, the two
PPMs come out R<->B swapped, and this tool names it. A whole-image mean would
report a small number and bury it; per-channel means plus the residual test
below make it unmissable.

The swap test compares mean|A - B| against mean|A - reverse(B)|. It also reports
whether a swap is DETECTABLE at all: on a neutral grey frame (R==G==B
everywhere -- e.g. the specular probe worlds) a channel swap is byte-identical
and CANNOT be seen. Those rows report `channel_swap: "UNDETERMINABLE"` rather
than "NONE", because "we did not see a swap" and "a swap would have been
invisible here" are different claims.

MECHANISM
---------
Arms are built as temp SIBLING copies of the world
(`.omnisim_sensor_oracle_<stem>_<arm>.omniworld`, same directory so the
controller and relative texture paths still resolve), with only the
`renderBackend` field inside each `Camera { ... }` block rewritten. The copies
are deleted on exit (`--keep` retains them). NOTE for the campaign owner: no
.gitignore rule covers that prefix yet, so a crashed run can leave one untracked
-- adding `**/.omnisim_sensor_oracle_*.omniworld` to .gitignore would close
that. This tool does not edit .gitignore.

Image capture reuses the shipped `camera_wgpu_smoke` controller, which dumps the
full readback as a P6 PPM when `OMNISIM_R33B_PPM_PATH` is set. That is why the
supported worlds are the ones already driving it.
`tests/api/worlds/camera*.omniworld` are NOT usable as-is and are not silently
adapted: they run `camera_checker` (which dumps no image), they need the
TestSuiteEmitter / TestSuiteSupervisor PROTO harness, `camera_color*.omniworld`
pulls its chart texture over https (violating the asset-locality rule), and a
bare pytest over tests/ rewrites worlds through the engine. Adapting them means
giving them an image-dumping controller first; that is a world / controller
change, not a flag on this script.

USAGE
-----
    # the instrument's own red-team check -- needs no binary, no GPU
    python scripts/dev/sensor_oracle.py --self-test

    # default corpus (7 camera probe worlds)
    python scripts/dev/sensor_oracle.py --json out.json

    # one world, keep the artefacts for eyeballing
    python scripts/dev/sensor_oracle.py \
        --world projects/samples/demos/worlds/rendering/camera_wgpu_golden.omniworld \
        --keep --verbose

Exit code 0 only if every row is MATCH. Any DIFFERS / CHANNEL_SWAP /
SIZE_MISMATCH / NO_IMAGE / WGPU_NOT_ENGAGED / WREN_ARM_CONTAMINATED /
UNSUPPORTED_WORLD is a non-zero exit.

TRAPS THIS ENCODES
------------------
 1. `OMNISIM_FORCE_WREN` / `OMNISIM_LEGACY` are PRESENCE-gated in the engine
    (`v != nullptr && v[0] != '\0'`), so `=0` ARMS them. Either one in the
    ambient environment pins BOTH arms to WREN and every row becomes a vacuous
    MATCH. The tool strips them from the child environment and says so.
 2. The PPM is written progressively. "The file exists" is not "the file is
    complete" -- wait for a stable size AND a full-length P6 payload.
 3. Scratch lives under `_scratch/` in the repo, not TEMP: 8.3 short-path
    mangling of a long Windows username has broken the controller's path
    resolution before (see wgpu_probe_golden.py).
 4. The first headless launch of a camera world is intermittently flaky (a Qt
    thread-destroy before the controller dumps). Retry -- do not read one empty
    run as a rendering failure.
 5. Give the child real std handles, not DEVNULL: on Windows DEVNULL can hand
    the spawned controller invalid handles and produce a "no result" launch.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PROBE_WORLD_DIR = REPO / "projects" / "samples" / "demos" / "worlds" / "rendering"

# The default corpus: every shipped camera probe world that renders a real scene
# through the `camera_wgpu_smoke` controller.
#
# Two probe worlds are deliberately EXCLUDED, with reasons, so nobody re-adds
# them expecting a signal:
#   camera_wgpu_smoke.omniworld -- no scene at all. The wgpu path renders its own
#       debug clear colour and WREN renders the world Background; they are
#       unrelated by construction, so a DIFFERS there measures nothing.
#   camera_wgpu_newton_delta.omniworld -- carries a physics-driven Husky on a
#       second controller. It is a legitimate row (the camera still reads at a
#       fixed step count) but it is slow and couples this instrument to physics
#       determinism. Pass it with --world if you want it.
DEFAULT_WORLDS = [
    "camera_wgpu_scene_smoke.omniworld",
    "camera_wgpu_golden.omniworld",
    "camera_wgpu_emissive_smoke.omniworld",
    "camera_wgpu_specular_smoke.omniworld",
    "camera_wgpu_specular_rough_smoke.omniworld",
    "camera_wgpu_shadow_cast.omniworld",
    "camera_wgpu_shadow_nocaster.omniworld",
]

TEMP_PREFIX = ".omnisim_sensor_oracle_"
DUMP_CONTROLLER = "camera_wgpu_smoke"
DEFAULT_CAMERA_NAME = "cam_wgpu"

# The positive wgpu marker OmCamera emits when its wgpu render+readback
# succeeded. Formatted as:
#   [OmCamera] 'cam_wgpu' rendered through wgpu (64x48, 1 draw)
WGPU_MARKER_RE = re.compile(r"\[OmCamera\]\s+'([^']*)'\s+rendered through wgpu")

# Matches `Camera {`, `DEF CAM Camera {`, `camera Camera {` -- never `lens Lens {`.
_NODE_HEADER_RE = re.compile(
    r"^(\s*)(?:DEF\s+\S+\s+)?(?:[A-Za-z_]\w*\s+)?(Camera|RangeFinder|Lidar)\s*\{")
# Trailing `# ...` comments are legal on a field line and DID break an earlier
# draft of this regex, which then silently inserted a second renderBackend.
_RENDER_BACKEND_RE = re.compile(r'^(\s*)renderBackend\s+"([^"]*)"[ \t]*(#.*)?$')
_NAME_FIELD_RE = re.compile(r'^\s*name\s+"([^"]*)"[ \t]*(?:#.*)?$')
_CONTROLLER_FIELD_RE = re.compile(r'^\s*controller\s+"([^"]*)"[ \t]*(?:#.*)?$')


# --------------------------------------------------------------------------
# world text surgery
# --------------------------------------------------------------------------

def brace_counts(line: str) -> tuple:
    """Count { and } in `line`, ignoring those inside "strings" or after a #.

    VRML/.omniworld comments run from an unquoted '#' to end of line, and node
    `info` / `title` strings routinely contain braces. Counting naively puts the
    block walker permanently out of phase.
    """
    opens = closes = 0
    in_string = False
    i = 0
    n = len(line)
    while i < n:
        c = line[i]
        if in_string:
            if c == "\\":
                i += 2
                continue
            if c == '"':
                in_string = False
        else:
            if c == '"':
                in_string = True
            elif c == "#":
                break
            elif c == "{":
                opens += 1
            elif c == "}":
                closes += 1
        i += 1
    return opens, closes


def _eol(line: str) -> str:
    if line.endswith("\r\n"):
        return "\r\n"
    if line.endswith("\n"):
        return "\n"
    return "\n"


def _iter_blocks(lines, node_types):
    """Yield (start, end_exclusive, node_type, depth_before) per matched block.

    `depth_before[k]` is the brace depth *entering* lines[start + k], relative to
    the block: 0 for the header line, 1 for a direct field of that node.

    Blocks are consumed whole, so a matched node nested inside another matched
    node is not re-visited -- fine here: a Camera inside a Camera is not a thing,
    while a Camera inside a Robot IS found, because `Robot {` is not a matched
    header and the walker steps over it one line at a time.
    """
    i = 0
    n = len(lines)
    while i < n:
        m = _NODE_HEADER_RE.match(lines[i])
        if not m or m.group(2) not in node_types:
            i += 1
            continue
        o, c = brace_counts(lines[i])
        depth = o - c
        depth_before = [0]
        j = i + 1
        while j < n and depth > 0:
            depth_before.append(depth)
            o2, c2 = brace_counts(lines[j])
            depth += o2 - c2
            j += 1
        yield i, j, m.group(2), depth_before
        i = j


def rewrite_render_backend(text, value, node_types=("Camera",)):
    """Set `renderBackend "<value>"` on every matched node block.

    Returns (new_text, nodes_touched). Replaces an existing declaration in place
    (preserving indent, trailing comment and line ending) or inserts one directly
    after the node header when the field is absent -- the schema default is
    "wren", so a world that never declares it still needs an explicit line for
    the wgpu arm.
    """
    lines = text.splitlines(keepends=True)
    edits = []  # (start, end, replacement_lines)
    touched = 0
    for start, end, _node_type, depth_before in _iter_blocks(lines, node_types):
        block = lines[start:end]
        hit = None
        for k in range(1, len(block)):
            if k < len(depth_before) and depth_before[k] == 1:
                m = _RENDER_BACKEND_RE.match(block[k].rstrip("\r\n"))
                if m:
                    hit = (k, m.group(1), m.group(3))
                    break
        new_block = list(block)
        if hit is not None:
            k, indent, comment = hit
            tail = ("   " + comment) if comment else ""
            new_block[k] = '%srenderBackend "%s"%s%s' % (indent, value, tail,
                                                         _eol(block[k]))
        else:
            header_indent = _NODE_HEADER_RE.match(block[0]).group(1)
            new_block.insert(1, '%s  renderBackend "%s"%s'
                                % (header_indent, value, _eol(block[0])))
        edits.append((start, end, new_block))
        touched += 1

    if not edits:
        return text, 0
    out = []
    cursor = 0
    for start, end, new_block in edits:
        out.extend(lines[cursor:start])
        out.extend(new_block)
        cursor = end
    out.extend(lines[cursor:])
    return "".join(out), touched


def scan_world(text):
    """Report what the oracle needs to know before it bothers booting a world."""
    lines = text.splitlines(keepends=True)
    cameras = []
    for start, end, node_type, depth_before in _iter_blocks(lines, ("Camera",)):
        block = lines[start:end]
        name = None
        backend = None
        for k in range(1, len(block)):
            if k >= len(depth_before) or depth_before[k] != 1:
                continue
            stripped = block[k].rstrip("\r\n")
            mn = _NAME_FIELD_RE.match(stripped)
            if mn:
                name = mn.group(1)
            mb = _RENDER_BACKEND_RE.match(stripped)
            if mb:
                backend = mb.group(2)
        cameras.append({"name": name if name is not None else "camera",
                        "declared_render_backend": backend,
                        "node_type": node_type})
    controllers = sorted({m.group(1) for m in
                          (_CONTROLLER_FIELD_RE.match(ln.rstrip("\r\n")) for ln in lines)
                          if m})
    return {"cameras": cameras, "controllers": controllers}


# --------------------------------------------------------------------------
# PPM I/O  (P6, 8-bit -- what camera_wgpu_smoke writes)
# --------------------------------------------------------------------------

def read_ppm(path: Path):
    """Read a binary P6 PPM; return (width, height, rgb_bytes).

    Raises ValueError on any malformed or short file, so a TRUNCATED dump is
    never mistaken for a frame (trap 2).
    """
    data = path.read_bytes()
    tokens = []
    pos = 0
    n = len(data)
    while len(tokens) < 4 and pos < n:
        c = data[pos:pos + 1]
        if c in b" \t\r\n":
            pos += 1
            continue
        if c == b"#":
            while pos < n and data[pos:pos + 1] not in b"\r\n":
                pos += 1
            continue
        start = pos
        while pos < n and data[pos:pos + 1] not in b" \t\r\n":
            pos += 1
        tokens.append(data[start:pos])
    if len(tokens) < 4:
        raise ValueError("%s: truncated PPM header" % path)
    if tokens[0] != b"P6":
        raise ValueError("%s: not a P6 PPM (magic %r)" % (path, tokens[0]))
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval != 255:
        raise ValueError("%s: only 8-bit PPM supported (maxval %d)" % (path, maxval))
    pos += 1  # exactly one whitespace byte separates header from raster
    want = width * height * 3
    rgb = data[pos:pos + want]
    if len(rgb) != want:
        raise ValueError("%s: short raster (%d of %d bytes) -- truncated dump"
                         % (path, len(rgb), want))
    return width, height, rgb


def write_ppm(path: Path, width: int, height: int, rgb: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (width, height))
        f.write(rgb)


def _np():
    try:
        import numpy as np
    except ImportError:
        sys.exit("sensor_oracle.py needs numpy (pip install numpy)")
    return np


def load_rgb(path: Path):
    np = _np()
    w, h, rgb = read_ppm(path)
    return np.frombuffer(rgb, dtype=np.uint8).reshape(h, w, 3)


# --------------------------------------------------------------------------
# comparison
# --------------------------------------------------------------------------

def _corr(x, y):
    """Pearson correlation, or None when either input has zero variance.

    A flat channel correlates with nothing; returning 0.0 there would imply we
    measured a disagreement we did not.
    """
    np = _np()
    xf = x.astype(np.float64).ravel()
    yf = y.astype(np.float64).ravel()
    xs = xf - xf.mean()
    ys = yf - yf.mean()
    den = float((xs * xs).sum()) * float((ys * ys).sum())
    if den <= 0.0:
        return None
    return round(float((xs * ys).sum() / (den ** 0.5)), 4)


def compare(a, b, threshold=30, swap_floor=2.0, swap_margin=0.5):
    """Compare two nominally-RGB frames. Every number here is measured.

    `threshold` is the per-pixel SUMMED-RGB absolute difference above which a
    pixel counts as a real change (matches render_ab.py's convention).
    `swap_floor` is the minimum mean|R-B| an arm must show for a channel swap to
    be detectable at all.
    """
    np = _np()
    ai = a.astype(np.int32)
    bi = b.astype(np.int32)
    d = np.abs(ai - bi)
    per_pixel = d.sum(axis=2)

    bi_rev = bi[:, :, ::-1]
    direct_mean = float(d.mean())
    swapped_mean = float(np.abs(ai - bi_rev).mean())

    spread_a = float(np.abs(ai[:, :, 0] - ai[:, :, 2]).mean())
    spread_b = float(np.abs(bi[:, :, 0] - bi[:, :, 2]).mean())
    detectable = max(spread_a, spread_b) >= swap_floor

    # A UNIFORM arm is not a reference. Measured 2026-08-22: the WREN arm of
    # camera_wgpu_scene_smoke came back solid black (mean RGB 0,0,0, std 0) because the
    # WREN camera does not dump under the flags this tool launches with -- and the run
    # still printed a confident "DIFFERS" against it, which is a comparison against
    # nothing. Flag a flat arm explicitly so a broken reference can never masquerade as a
    # measured difference. (It is only a WARNING, not a hard failure: a deliberately flat
    # test pattern is legitimate, and the caller can judge from `flat_arms`.)
    flat = []
    if float(ai.std()) < 1e-6:
        flat.append({"arm": "A", "mean_rgb": [float(x) for x in ai.reshape(-1, 3).mean(axis=0)]})
    if float(bi.std()) < 1e-6:
        flat.append({"arm": "B", "mean_rgb": [float(x) for x in bi.reshape(-1, 3).mean(axis=0)]})

    if not detectable:
        swap_verdict = "UNDETERMINABLE"
        swap_note = ("both arms are channel-neutral (mean|R-B| = %.3f / %.3f, below "
                     "the %.1f floor): an R<->B swap would be byte-identical here, so "
                     "this world cannot answer the question"
                     % (spread_a, spread_b, swap_floor))
    elif direct_mean >= 1.0 and swapped_mean < direct_mean * swap_margin:
        swap_verdict = "SWAPPED"
        swap_note = ("mean|A-B| = %.4f but mean|A-reverse(B)| = %.4f: arm B's red and "
                     "blue channels match arm A's blue and red -- a channel-order "
                     "difference, not a shading difference"
                     % (direct_mean, swapped_mean))
    else:
        swap_verdict = "NONE"
        swap_note = ("mean|A-B| = %.4f vs mean|A-reverse(B)| = %.4f: reversing B does "
                     "not improve the fit, so the channel order agrees"
                     % (direct_mean, swapped_mean))

    return {
        "same_size": True,
        "arm_a_mean_rgb": [round(float(ai[:, :, c].mean()), 3) for c in range(3)],
        "arm_b_mean_rgb": [round(float(bi[:, :, c].mean()), 3) for c in range(3)],
        "per_channel_mean_abs": [round(float(d[:, :, c].mean()), 4) for c in range(3)],
        "per_channel_max_abs": [int(d[:, :, c].max()) for c in range(3)],
        "mean_abs_summed": round(float(per_pixel.mean()), 4),
        "max_abs_summed": int(per_pixel.max()),
        "pixels_over_threshold": int((per_pixel > threshold).sum()),
        "threshold": threshold,
        "total_pixels": int(per_pixel.shape[0] * per_pixel.shape[1]),
        "flat_arms": flat,  # a uniform arm is not a reference -- see the guard above
        "channel_swap": swap_verdict,
        "channel_swap_note": swap_note,
        "channel_swap_detectable": bool(detectable),
        "mean_abs_direct": round(direct_mean, 4),
        "mean_abs_reversed": round(swapped_mean, 4),
        "arm_a_rb_spread": round(spread_a, 4),
        "arm_b_rb_spread": round(spread_b, 4),
        "corr_aR_bR": _corr(ai[:, :, 0], bi[:, :, 0]),
        "corr_aB_bB": _corr(ai[:, :, 2], bi[:, :, 2]),
        "corr_aR_bB": _corr(ai[:, :, 0], bi[:, :, 2]),
        "corr_aB_bR": _corr(ai[:, :, 2], bi[:, :, 0]),
    }


# --------------------------------------------------------------------------
# engine driving
# --------------------------------------------------------------------------

def find_binary() -> Path:
    home = Path(os.environ.get("OMNISIM_HOME") or REPO)
    for cand in (home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
                 home / "bin" / "omnisim-bin",
                 home / "Contents" / "MacOS" / "omnisim"):
        if cand.exists():
            return cand
    sys.exit("omnisim-bin not found under %s (set OMNISIM_HOME to the checkout)" % home)


def _wait_for_ppm(path: Path, deadline: float, proc):
    """Return (w, h, rgb) once the dump is COMPLETE, else None.

    Trap 2: the controller writes header-then-raster, so an existing file can
    still be half a frame. Require a stable size AND a parse that yields the
    full raster.
    """
    last = -1
    stable = 0
    while time.time() < deadline:
        if path.exists():
            size = path.stat().st_size
            if size > 0 and size == last:
                stable += 1
                if stable >= 2:
                    try:
                        return read_ppm(path)
                    except ValueError:
                        stable = 0  # still growing, or genuinely malformed
            else:
                stable = 0
            last = size
        elif proc.poll() is not None:
            # the engine is gone and never dumped; no point burning the budget
            time.sleep(0.3)
            if not path.exists():
                return None
        time.sleep(0.25)
    return None


def run_arm(binary: Path, world: Path, arm: str, out_dir: Path, extra_env,
            timeout_s: float, attempts: int, verbose: bool):
    """Boot `world` headless once per attempt until the PPM lands. Never raises."""
    stem = "%s__%s" % (world.stem, arm)
    ppm_path = out_dir / ("%s.ppm" % stem)
    result_path = out_dir / ("%s.result.txt" % stem)
    log_path = out_dir / ("%s.log" % stem)
    proc_path = out_dir / ("%s.proc.txt" % stem)

    row = {"arm": arm, "world_file": str(world), "ppm": str(ppm_path),
           "log": str(log_path), "captured": False, "attempts_used": 0,
           "errors": 0, "warnings": 0, "wgpu_marker_hits": 0,
           "wgpu_marker_cameras": [], "result_text": "", "size": None}

    home = Path(os.environ.get("OMNISIM_HOME") or REPO)
    # ⚠ RENDERING MUST BE ENABLED, and this is not a style choice.
    # This tool originally launched with "--minimize --batch --no-rendering --mode=fast", and the
    # WREN arm came back SOLID BLACK (mean RGB 0,0,0) on a world whose wgpu arm rendered fine.
    # That is a KNOWN, documented engine limitation, not a tool bug: tests/smoke skips
    # "rendering-normals" with the note that "Offscreen Camera framebuffers under --no-rendering
    # contain only the viewport clear color (=Background.skyColor), not the rendered scene
    # geometry", because wr_scene_render_to_viewports() does not include scene geometry while the
    # main 3D view's render loop is suppressed. The wgpu camera path does NOT share that defect
    # (it renders to its own offscreen target), which is exactly why the asymmetry is so easy to
    # mistake for a real difference: one arm renders, the other returns the clear colour.
    # So: no --no-rendering, and no --minimize. OMNISIM_SENSOR_ORACLE_FAST=1 restores the old
    # flags for an A/B of this very decision.
    #
    # ⚠ MEASURED 2026-08-22: dropping --no-rendering was NOT sufficient — the WREN arm is still
    # solid black under --batch. The WREN camera evidently needs a real window, which makes a
    # WREN reference expensive to obtain and is why `flat_arms` exists. FOR COLOUR CORRECTNESS
    # THERE IS A BETTER ORACLE THAN WREN ANYWAY: the world's OWN AUTHORED MATERIAL COLOUR. That
    # is how the R/B swap was actually proven — camera_wgpu_scene_smoke authors
    # `baseColor 0 1 1` (pure cyan, ZERO red) and the wgpu camera handed the controller
    # R=141,G=141,B=0. Authored colour is exact, free, and immune to the reference arm being
    # broken. Use WREN as the reference for SHADING questions, not for channel order.
    fast_arm = (os.environ.get("OMNISIM_SENSOR_ORACLE_FAST") or "").strip() not in ("", "0")
    cmd = [str(binary), str(world), "--batch", "--mode=fast", "--stdout", "--stderr"]
    if fast_arm:
        cmd = [str(binary), str(world), "--minimize", "--batch", "--no-rendering",
               "--mode=fast", "--stdout", "--stderr"]

    for attempt in range(1, attempts + 1):
        row["attempts_used"] = attempt
        for stale in (ppm_path, result_path, log_path, proc_path):
            try:
                stale.unlink()
            except FileNotFoundError:
                pass

        env = os.environ.copy()
        # Trap 1: either of these pins EVERY backend resolution to WREN, which
        # would make the wgpu arm secretly the WREN arm. They are presence-gated
        # in the engine, so "=0" does NOT disable them -- they must be absent.
        for poison in ("OMNISIM_FORCE_WREN", "OMNISIM_LEGACY"):
            env.pop(poison, None)
        env["OMNISIM_HOME"] = str(home)
        env["OMNISIM_LOG_PATH"] = str(log_path)
        env["OMNISIM_R33B_PPM_PATH"] = str(ppm_path)
        env["OMNISIM_R33B_RESULT_PATH"] = str(result_path)
        mingw = home / "msys64" / "mingw64" / "bin"
        env["PATH"] = (str(mingw) + os.pathsep + str(binary.parent) + os.pathsep
                       + env.get("PATH", ""))
        env.update(extra_env or {})

        # Trap 5: a real file, not DEVNULL.
        with proc_path.open("w", encoding="utf-8", errors="replace") as proc_out:
            proc = subprocess.Popen(cmd, cwd=str(REPO), env=env,
                                    stdout=proc_out, stderr=subprocess.STDOUT)
            frame = _wait_for_ppm(ppm_path, time.time() + timeout_s, proc)
            try:
                proc.terminate()
                proc.wait(timeout=10)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

        log_text = log_path.read_text(errors="replace") if log_path.exists() else ""
        hits = WGPU_MARKER_RE.findall(log_text)
        row["wgpu_marker_hits"] = len(hits)
        row["wgpu_marker_cameras"] = sorted(set(hits))
        row["errors"] = sum(1 for ln in log_text.splitlines() if ln.startswith("ERROR"))
        row["warnings"] = sum(1 for ln in log_text.splitlines()
                              if ln.startswith("WARNING"))
        if result_path.exists():
            row["result_text"] = result_path.read_text(errors="replace").strip()
        if frame is not None:
            row["captured"] = True
            row["size"] = [frame[0], frame[1]]
            return row
        if verbose:
            print("        %s arm attempt %d/%d: no complete PPM, retrying"
                  % (arm, attempt, attempts))
    return row


# --------------------------------------------------------------------------
# per-world driver
# --------------------------------------------------------------------------

def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def run_world(binary: Path, world: Path, out_dir: Path, args):
    row = {"world": _rel(world)}
    try:
        source = world.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        row["verdict"] = "UNSUPPORTED_WORLD"
        row["reason"] = "unreadable: %s" % exc
        return row

    info = scan_world(source)
    row["cameras"] = info["cameras"]
    row["controllers"] = info["controllers"]

    if not info["cameras"]:
        row["verdict"] = "UNSUPPORTED_WORLD"
        row["reason"] = "no Camera node found"
        return row
    if args.check_preconditions:
        names = [c["name"] for c in info["cameras"]]
        if args.camera_name not in names:
            row["verdict"] = "UNSUPPORTED_WORLD"
            row["reason"] = ("no Camera named %r (found %r); the %s controller reads "
                             "that device by name. Use --camera-name, or "
                             "--no-precondition-check if this world's own controller "
                             "dumps to OMNISIM_R33B_PPM_PATH."
                             % (args.camera_name, names, DUMP_CONTROLLER))
            return row
        if DUMP_CONTROLLER not in info["controllers"]:
            row["verdict"] = "UNSUPPORTED_WORLD"
            row["reason"] = ("no robot runs the %r controller (found %r); nothing in "
                             "this world dumps the controller-visible image. See the "
                             "module docstring on why tests/api camera worlds are not "
                             "silently adapted."
                             % (DUMP_CONTROLLER, info["controllers"]))
            return row

    arms = {}
    temp_worlds = []
    try:
        for arm in ("wren", "wgpu"):
            text, touched = rewrite_render_backend(source, arm)
            if touched != len(info["cameras"]):
                row["verdict"] = "UNSUPPORTED_WORLD"
                row["reason"] = ("renderBackend rewrite touched %d of %d Camera blocks "
                                 "-- refusing to run a half-flipped world"
                                 % (touched, len(info["cameras"])))
                return row
            tw = world.parent / ("%s%s_%s.omniworld" % (TEMP_PREFIX, world.stem, arm))
            tw.write_text(text, encoding="utf-8")
            temp_worlds.append(tw)
            print("   %-4s arm: %s" % (arm, tw.name))
            arms[arm] = run_arm(binary, tw, arm, out_dir, {},
                                args.timeout, args.attempts, args.verbose)
            print("        captured=%s size=%s errors=%d warnings=%d wgpu_marker=%d"
                  % (arms[arm]["captured"], arms[arm]["size"], arms[arm]["errors"],
                     arms[arm]["warnings"], arms[arm]["wgpu_marker_hits"]))
    finally:
        if not args.keep:
            for tw in temp_worlds:
                try:
                    tw.unlink()
                except OSError:
                    pass

    row["arm_wren"] = arms.get("wren")
    row["arm_wgpu"] = arms.get("wgpu")
    row["wren_arm_evidence"] = "absence_of_wgpu_marker"

    # --- evidence gates come FIRST: they invalidate any pixel verdict --------
    if arms["wgpu"]["wgpu_marker_hits"] == 0:
        row["verdict"] = "WGPU_NOT_ENGAGED"
        row["reason"] = ('the wgpu arm never logged "[OmCamera] ... rendered through '
                         'wgpu", so it fell back to WREN (no wgpu-native in this '
                         'build/host, or the readback failed). Both arms are WREN and '
                         'any pixel comparison would be vacuous.')
        return row
    if arms["wren"]["wgpu_marker_hits"] > 0:
        row["verdict"] = "WREN_ARM_CONTAMINATED"
        row["reason"] = ("the wren arm logged the wgpu marker for camera(s) %r -- the "
                         "renderBackend rewrite did not take, or another Camera in "
                         "this world is pinned to wgpu."
                         % (arms["wren"]["wgpu_marker_cameras"],))
        return row

    missing = [a for a in ("wren", "wgpu") if not arms[a]["captured"]]
    if missing:
        row["verdict"] = "NO_IMAGE"
        row["reason"] = ("no complete PPM from arm(s): %s (after %d attempts each). "
                         "This is an INSTRUMENT result, not a parity verdict."
                         % (", ".join(missing), args.attempts))
        return row
    if arms["wren"]["size"] != arms["wgpu"]["size"]:
        row["verdict"] = "SIZE_MISMATCH"
        row["diff"] = {"same_size": False,
                       "arm_a_size": arms["wren"]["size"],
                       "arm_b_size": arms["wgpu"]["size"]}
        return row

    a = load_rgb(Path(arms["wren"]["ppm"]))
    b = load_rgb(Path(arms["wgpu"]["ppm"]))
    d = compare(a, b, args.threshold, args.swap_floor, args.swap_margin)
    row["diff"] = d
    if d["channel_swap"] == "SWAPPED":
        row["verdict"] = "CHANNEL_SWAP"
    elif d["pixels_over_threshold"] > 0:
        row["verdict"] = "DIFFERS"
    else:
        row["verdict"] = "MATCH"
    return row


def print_row(row) -> None:
    d = row.get("diff")
    if d and d.get("same_size"):
        print("        WREN mean RGB=%s   wgpu mean RGB=%s"
              % (d["arm_a_mean_rgb"], d["arm_b_mean_rgb"]))
        print("        per-channel mean|d|=%s  max|d|=%s"
              % (d["per_channel_mean_abs"], d["per_channel_max_abs"]))
        print("        summed mean=%s max=%s  px>%d = %d of %d"
              % (d["mean_abs_summed"], d["max_abs_summed"], d["threshold"],
                 d["pixels_over_threshold"], d["total_pixels"]))
        for fa in (d.get("flat_arms") or []):
            print("        !! arm %s is UNIFORM (mean RGB %s) -- a flat arm is not a reference; "
                  "any verdict against it is a comparison against nothing"
                  % (fa["arm"], fa["mean_rgb"]))
        print("        channel-swap: %s -- %s"
              % (d["channel_swap"], d["channel_swap_note"]))
        print("        corr R:R=%s B:B=%s | R:B=%s B:R=%s"
              % (d["corr_aR_bR"], d["corr_aB_bB"], d["corr_aR_bB"], d["corr_aB_bR"]))
    print("   VERDICT: %s%s" % (row["verdict"],
                                ("  -- " + row["reason"]) if row.get("reason") else ""))


# --------------------------------------------------------------------------
# self-test -- the instrument must be able to go RED
# --------------------------------------------------------------------------

def self_test(out_dir: Path) -> int:
    """Prove this tool reports a channel swap, a real difference, a truncated
    dump and a world-rewrite miss -- and that it does NOT claim a swap verdict on
    a frame where a swap would be invisible. Needs no binary and no GPU."""
    np = _np()
    out_dir.mkdir(parents=True, exist_ok=True)
    failures = []

    def check(name, ok, detail):
        print("[self-test] %-4s %s: %s" % ("PASS" if ok else "FAIL", name, detail))
        if not ok:
            failures.append(name)

    h, w = 24, 32
    rng = np.random.RandomState(1234)
    # A deliberately COLOURFUL frame: R and B must differ, or a swap is invisible.
    base = np.zeros((h, w, 3), dtype=np.uint8)
    base[:, :, 0] = rng.randint(0, 90, size=(h, w))     # low red
    base[:, :, 1] = rng.randint(60, 160, size=(h, w))   # mid green
    base[:, :, 2] = rng.randint(170, 256, size=(h, w))  # high blue

    # 1. PPM round-trip (the capture format itself is a failure surface).
    p = out_dir / "selftest_base.ppm"
    write_ppm(p, w, h, base.tobytes())
    rt = load_rgb(p)
    check("ppm_roundtrip", bool(np.array_equal(rt, base)),
          "wrote+read %dx%d P6, arrays %s"
          % (w, h, "identical" if np.array_equal(rt, base) else "DIFFER"))

    # 2. Identical frames -> MATCH, zero difference, swap NONE (not SWAPPED).
    d = compare(base, base.copy(), threshold=30)
    ok = (d["pixels_over_threshold"] == 0 and d["mean_abs_summed"] == 0.0
          and d["channel_swap"] == "NONE" and d["channel_swap_detectable"])
    check("identical_is_match", ok,
          "px>30=%d mean=%s swap=%s" % (d["pixels_over_threshold"],
                                        d["mean_abs_summed"], d["channel_swap"]))

    # 3. THE RED CASE: an R<->B swapped copy must be reported as SWAPPED.
    swapped = base[:, :, ::-1].copy()
    d = compare(base, swapped, threshold=30)
    ok = (d["channel_swap"] == "SWAPPED" and d["mean_abs_reversed"] == 0.0
          and d["mean_abs_direct"] > 0.0 and d["corr_aR_bB"] == 1.0)
    check("channel_swap_detected", ok,
          "swap=%s direct=%s reversed=%s corr R:B=%s (vs R:R=%s)"
          % (d["channel_swap"], d["mean_abs_direct"], d["mean_abs_reversed"],
             d["corr_aR_bB"], d["corr_aR_bR"]))

    # 4. A swap on a NEUTRAL frame is byte-identical -> the tool must say
    #    UNDETERMINABLE, never "NONE" (which would claim we checked).
    grey = np.repeat(rng.randint(0, 256, size=(h, w, 1)).astype(np.uint8), 3, axis=2)
    d = compare(grey, grey[:, :, ::-1].copy(), threshold=30)
    ok = (d["channel_swap"] == "UNDETERMINABLE" and not d["channel_swap_detectable"])
    check("neutral_swap_undeterminable", ok,
          "swap=%s detectable=%s spread=%s/%s"
          % (d["channel_swap"], d["channel_swap_detectable"],
             d["arm_a_rb_spread"], d["arm_b_rb_spread"]))

    # 5. Sub-threshold noise must NOT be called a difference...
    near = np.clip(base.astype(np.int32) + 1, 0, 255).astype(np.uint8)
    d = compare(base, near, threshold=30)
    ok = d["pixels_over_threshold"] == 0 and d["mean_abs_summed"] > 0.0
    check("subthreshold_is_match", ok,
          "mean=%s px>30=%d" % (d["mean_abs_summed"], d["pixels_over_threshold"]))

    # 6. ...but a real localised change must be.
    changed = base.copy()
    changed[4:10, 4:10, :] = np.clip(changed[4:10, 4:10, :].astype(np.int32) + 80,
                                     0, 255).astype(np.uint8)
    d = compare(base, changed, threshold=30)
    ok = d["pixels_over_threshold"] == 36 and d["channel_swap"] == "NONE"
    check("real_difference_detected", ok,
          "px>30=%d (expect 36) max=%d swap=%s"
          % (d["pixels_over_threshold"], d["max_abs_summed"], d["channel_swap"]))

    # 7. A truncated PPM must raise, not be read as a frame.
    trunc = out_dir / "selftest_truncated.ppm"
    full = p.read_bytes()
    trunc.write_bytes(full[: len(full) // 2])
    try:
        read_ppm(trunc)
        check("truncated_ppm_rejected", False, "read a half-written dump as a frame")
    except ValueError as exc:
        check("truncated_ppm_rejected", True, str(exc).split(": ", 1)[-1])

    # 8. World rewrite: flips a declared Camera field (even with a trailing
    #    comment), INSERTS a missing one inside the right block, and never
    #    touches a Viewpoint that declares the same field name.
    sample = (
        '#OMNISIM R2025a utf8\n'
        'WorldInfo {\n'
        '  info [ "a brace } in a string, and a { too" ]\n'
        '}\n'
        'Viewpoint {\n'
        '  renderBackend "wgpu"\n'
        '}\n'
        'Robot {\n'
        '  children [\n'
        '    Camera {\n'
        '      name "cam_wgpu"\n'
        '      zoom Zoom {\n'
        '        maxFieldOfView 1.58\n'
        '      }\n'
        '      renderBackend "wgpu"   # comment { not a brace\n'
        '    }\n'
        '    DEF SECOND Camera {\n'
        '      name "cam2"\n'
        '    }\n'
        '  ]\n'
        '  controller "camera_wgpu_smoke"\n'
        '}\n'
    )
    scanned = scan_world(sample)
    ok = ([c["name"] for c in scanned["cameras"]] == ["cam_wgpu", "cam2"]
          and scanned["controllers"] == ["camera_wgpu_smoke"])
    check("scan_world", ok, "cameras=%s controllers=%s"
          % ([c["name"] for c in scanned["cameras"]], scanned["controllers"]))

    wren_text, touched = rewrite_render_backend(sample, "wren")
    ok = (touched == 2
          and wren_text.count('renderBackend "wren"') == 2
          and 'Viewpoint {\n  renderBackend "wgpu"' in wren_text
          and wren_text.count('renderBackend "wgpu"') == 1)  # only the Viewpoint's
    check("rewrite_wren_arm", ok,
          "touched=%d wren-lines=%d wgpu-lines=%d (the Viewpoint must keep its own)"
          % (touched, wren_text.count('renderBackend "wren"'),
             wren_text.count('renderBackend "wgpu"')))

    wgpu_text, touched = rewrite_render_backend(sample, "wgpu")
    ok = (touched == 2 and wgpu_text.count('renderBackend "wgpu"') == 3
          and 'renderBackend "wren"' not in wgpu_text)
    check("rewrite_wgpu_arm", ok,
          "touched=%d wgpu-lines=%d (2 cameras + the untouched Viewpoint)"
          % (touched, wgpu_text.count('renderBackend "wgpu"')))

    # The trailing comment must survive, and the insert must land INSIDE the
    # second Camera block rather than after it.
    check("rewrite_keeps_comment",
          '# comment { not a brace' in wren_text,
          "trailing comment preserved on the rewritten field line")
    second = wgpu_text.split("DEF SECOND Camera {", 1)[1].split("}", 1)[0]
    check("rewrite_inserts_in_block", 'renderBackend "wgpu"' in second,
          "second Camera block now: %r" % second.strip().replace("\n", " | "))

    # Brace counting must ignore strings and comments.
    tricky = '  info [ "a { brace }" ]  # and a { comment'
    check("brace_counts_ignores_strings", brace_counts(tricky) == (0, 0),
          "counted %s for a line whose only braces are quoted or commented"
          % (brace_counts(tricky),))

    print()
    if failures:
        print("[self-test] %d case(s) FAILED: %s" % (len(failures), ", ".join(failures)))
        print("[self-test] the instrument is NOT trustworthy -- fix it before "
              "believing any parity verdict it prints.")
        return 1
    print("[self-test] all cases pass: the comparator goes RED on a channel swap and "
          "on a real difference, refuses a truncated dump, declines to claim a swap "
          "verdict it cannot see, and rewrites only Camera blocks.")
    return 0


# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", action="append", default=[],
                    help="repo-relative or absolute world path (repeatable). "
                         "Default: the 7 shipped camera probe worlds.")
    ap.add_argument("--camera-name", default=DEFAULT_CAMERA_NAME,
                    help="device name the dumping controller reads (default %s)"
                         % DEFAULT_CAMERA_NAME)
    ap.add_argument("--no-precondition-check", dest="check_preconditions",
                    action="store_false", default=True,
                    help="skip the 'world has a <camera-name> Camera driven by the "
                         "%s controller' check -- for a world with its own "
                         "OMNISIM_R33B_PPM_PATH-compatible controller" % DUMP_CONTROLLER)
    ap.add_argument("--threshold", type=int, default=30,
                    help="per-pixel summed-RGB |difference| counted as a REAL change "
                         "(default 30, matching render_ab.py)")
    ap.add_argument("--swap-floor", type=float, default=2.0,
                    help="minimum mean|R-B| within an arm for a channel swap to be "
                         "detectable at all (default 2.0)")
    ap.add_argument("--swap-margin", type=float, default=0.5,
                    help="the reversed residual must be below margin*direct to call a "
                         "swap (default 0.5)")
    ap.add_argument("--attempts", type=int, default=3,
                    help="retries per arm for the flaky first headless load")
    ap.add_argument("--timeout", type=float, default=45.0,
                    help="seconds to wait for one arm's PPM dump")
    ap.add_argument("--out-dir", default=None,
                    help="where PPMs/logs land (default _scratch/sensor_oracle -- "
                         "NOT TEMP, see trap 3)")
    ap.add_argument("--keep", action="store_true",
                    help="keep the temp sibling world files for inspection")
    ap.add_argument("--json", default=None)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--self-test", action="store_true",
                    help="prove the comparator can go RED; no binary/GPU needed")
    args = ap.parse_args()

    out_dir = Path(args.out_dir) if args.out_dir else REPO / "_scratch" / "sensor_oracle"
    if args.self_test:
        return self_test(out_dir / "selftest")

    for poison in ("OMNISIM_FORCE_WREN", "OMNISIM_LEGACY"):
        if os.environ.get(poison):
            print("!! %s is set in this shell. It is PRESENCE-gated in the engine "
                  "(=0 ARMS it) and pins BOTH arms to WREN. Stripping it from the "
                  "child environment." % poison)

    binary = find_binary()
    print("[sensor-oracle] binary: %s" % binary)
    out_dir.mkdir(parents=True, exist_ok=True)

    worlds = [Path(w) if Path(w).is_absolute() else REPO / w for w in args.world]
    if not worlds:
        worlds = [PROBE_WORLD_DIR / name for name in DEFAULT_WORLDS]

    # Clear sibling worlds left by a crashed earlier run -- our prefix only,
    # because another agent may be working in this tree.
    for w in worlds:
        for stale in w.parent.glob("%s*.omniworld" % TEMP_PREFIX):
            try:
                stale.unlink()
            except OSError:
                pass

    results = []
    for world in worlds:
        print("== %s" % world.name)
        if not world.exists():
            results.append({"world": _rel(world), "verdict": "UNSUPPORTED_WORLD",
                            "reason": "world not found"})
            print("   VERDICT: UNSUPPORTED_WORLD  -- world not found")
            continue
        row = run_world(binary, world, out_dir, args)
        print_row(row)
        results.append(row)

    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=1), encoding="utf-8")
        print("\nwrote %s" % args.json)

    matched = [r for r in results if r.get("verdict") == "MATCH"]
    print("\n%d/%d worlds MATCH (WREN camera bytes == wgpu camera bytes)"
          % (len(matched), len(results)))
    for r in results:
        if r.get("verdict") != "MATCH":
            print("  !! %s: %s" % (Path(r["world"]).name, r["verdict"]))
    return 0 if results and len(matched) == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
