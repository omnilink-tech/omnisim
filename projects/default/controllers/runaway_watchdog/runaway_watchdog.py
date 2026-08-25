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

"""runaway_watchdog — the headless lane's physical-plausibility sampler.

`python -m omnisim run-headless <world> --fail-on-runaway` injects this Robot
into a sibling copy of the world. It is a DUMB SAMPLER on purpose: it appends
one JSON line per sample and never decides anything. The verdict is computed
by `scripts/dev/headless_runner.py:runaway_verdict()` so the physics assertion
is unit-testable without an engine (tests/harness/test_headless_verdicts.py).

Why it exists: the headless lane's PASS meant "the world loaded, finalised and
stepped, and logged no ERROR/WARNING". It said nothing about whether the result
is physically possible, so a world whose floor has no `boundingObject` — a
dynamic body free-falling for ever through a hologram floor — passed
identically to the fixed world (measured on the AgentBench C2 task, both
variants PASS, 0 errors, 0 warnings). Nothing in the engine log records a body
leaving the world, so the only honest way to know is to sample poses.

Output (`--out=PATH`, JSONL, flushed per line so a terminated run still has
everything up to the kill):

    {"kind":"header","dt_ms":16,"period_steps":8,
     "bodies":[{key,name,type,via}],"skipped":[{key,type,reason}],
     "skipped_total":N,...}
    {"kind":"floor","static_floor_z":<float|null>,"static_bodies":N,
     "nodes_visited":N,"truncated":<bool>,"scan_ms":N}
    {"kind":"s","t":<sim seconds>,"z":{"<key>":[x,y,z], ...}}
    {"kind":"end","t":...,"samples":N}            (only on a clean stop)

Controller args (all optional):
    --out=PATH            JSONL path (default $OMNISIM_RUNAWAY_OUT, else
                          runaway_watchdog.jsonl in the world's directory)
    --period-steps=N      sample every N basic timesteps (default 8)
    --max-samples=N       stop sampling after N samples (default 20000)
    --scan-budget-s=S     wall-clock ceiling for the one-off static-collider
                          scan (default 4.0; half of it caps the tree walk)

Cost: one IPC round trip per N basic steps plus one getPosition() per tracked
top-level dynamic body. Nothing is enumerated per step: the roster and the
static floor reference are computed once, at t=0, and the scan is bounded in
both node count and wall clock because EVERY field read is an IPC round trip.
"""

from __future__ import annotations

import json
import os
import sys
import time
import traceback

from omnisim import Supervisor

# Bodies are classified by FIELD, never by type name: a world's floor is often a
# PROTO (`Floor {}`, `RectangleArena {}`, `UnevenTerrain {}`) whose type name is
# not "Solid", and the two questions that matter have field answers --
#   dynamic body      = carries a Physics node
#   static collider   = carries a boundingObject and NO Physics node
# (Measured: a type-name walk found 0 static bodies in warehouse_husky.omniworld,
# whose floor is `Floor {}`.)
#
# Never track the injected helpers themselves.
EXCLUDED_NAMES = ("runaway_watchdog", "harness_supervisor", "agentbench_recorder")


def parse_args(argv):
    out = {}
    for a in argv:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            out[k] = v
    return out


def _sf_string(node, name):
    f = node.getField(name) if node is not None else None
    if f is None:
        return ""
    try:
        return f.getSFString() or ""
    except Exception:  # noqa: BLE001
        return ""


def _typename(node):
    try:
        return node.getTypeName()
    except Exception:  # noqa: BLE001
        return ""


def _sf_node(node, name):
    f = node.getField(name) if node is not None else None
    if f is None:
        return None
    try:
        return f.getSFNode()
    except Exception:  # noqa: BLE001
        return None


def _has_physics(node):
    """True when the node carries a Physics node, i.e. is a dynamic body."""
    return _sf_node(node, "physics") is not None


def _children(node, limit=256, deadline=None):
    """Child nodes, capped. `limit`/`deadline` matter: one getMFNode() is one IPC
    round trip, and a procedural world can hang thousands of children off a
    single Group -- an uncapped read of one such node costs more than the whole
    scan budget."""
    f = node.getField("children") if node is not None else None
    if f is None:
        return []
    try:
        count = min(f.getCount(), limit)
    except Exception:  # noqa: BLE001
        return []
    out = []
    for i in range(count):
        if deadline is not None and time.monotonic() > deadline:
            break
        try:
            child = f.getMFNode(i)
        except Exception:  # noqa: BLE001
            continue
        if child is not None:
            out.append(child)
    return out


def _def_of(node):
    try:
        return node.getDef() or ""
    except Exception:  # noqa: BLE001
        return ""


# Grouping nodes carry no Physics of their own, but their children can be
# ordinary dynamic bodies. A world that wraps its crates in a `Group` (or the
# `Pose`/`Transform` an editor emits) used to yield a roster of ZERO -- and a
# roster of zero makes every sample row empty, which the runner read as
# "nothing left the world" instead of "nothing was measured". One level of
# descent covers the ordinary authoring shapes for one extra getMFNode() per
# container. It is ONE level on purpose: the cost argument below is unchanged,
# and the roster (plus `skipped`) is reported in the header so the coverage
# stays visible rather than assumed.
GROUPING_TYPES = ("Group", "Transform", "Pose")


def _top_level_bodies(supervisor, max_descend=64):
    """Dynamic bodies (nodes carrying a Physics node) near the top of the scene.

    Root's direct children, PLUS one level down through the grouping nodes
    (`Group` / `Transform` / `Pose`) that carry no physics themselves.

    Deliberately SHALLOW: a wheel or a link that "escapes" does so because its
    parent did (or because a joint broke, which is a different defect), and one
    getPosition() per tracked body per sample is one IPC round trip -- walking
    every descendant of a 100-body warehouse would cost more than the run it is
    measuring.

    Returns ``(bodies, skipped)`` where `bodies` is a list of
    ``(node, container_or_None)`` and `skipped` names every walked root child
    that was NOT tracked, with the reason. The runner prints `skipped` when the
    roster comes back empty, so "no bodies" is explainable instead of silent.
    """
    root = supervisor.getRoot()
    dynamic: list = []
    skipped: list = []
    descended = 0
    for node in _children(root):
        if _sf_string(node, "name") in EXCLUDED_NAMES:
            continue
        if _has_physics(node):
            dynamic.append((node, None))
            continue
        type_name = _typename(node)
        found = 0
        if type_name in GROUPING_TYPES and descended < max_descend:
            for child in _children(node, limit=max_descend - descended):
                descended += 1
                if _sf_string(child, "name") in EXCLUDED_NAMES:
                    continue
                if _has_physics(child):
                    dynamic.append((child, node))
                    found += 1
        if found:
            continue
        skipped.append({
            "key": _def_of(node) or _sf_string(node, "name") or type_name or "?",
            "type": type_name,
            "reason": ("no Physics node here and none one level down: a static "
                       "collider, a visual-only node, or a container whose "
                       "bodies are nested deeper (a supervisor cannot see "
                       "inside a PROTO)"),
        })
    return dynamic, skipped


def _static_colliders(supervisor, deadline, max_visits=400):
    """Nodes with a boundingObject and no Physics node, within a scan budget.

    That pair IS the definition of a static collision surface, and it holds for
    PROTO floors whose type name is not "Solid". BOUNDED on purpose: every
    getField()/getSFNode() is an IPC round trip to the engine, so an unbounded
    walk of a procedural world is not "a bit slow", it never finishes inside the
    run (measured: on distribution/generated_worlds/mars.wbt an unbounded walk
    had not even emitted the header after 25 s and the check reported NO
    SAMPLES). Truncation is reported, never hidden.
    """
    out, visits, truncated = [], 0, False
    stack = list(_children(supervisor.getRoot(), deadline=deadline))
    while stack:
        if visits >= max_visits or time.monotonic() > deadline:
            truncated = True
            break
        node = stack.pop()
        visits += 1
        if _sf_node(node, "boundingObject") is not None and not _has_physics(node):
            out.append(node)
        else:
            # Only descend through nodes that are NOT themselves a collider: a
            # floor's or a wall's internals cannot lower the world's floor.
            stack.extend(_children(node, deadline=deadline))
    return out, visits, truncated


def _floor_reference(static_nodes, deadline):
    """Lowest world-space z of any static collider's geometry, or None.

    It has to come from geometry, not from the node's origin: a 20x20x0.1 floor
    Solid sits at z=0 with its underside at z=-0.05. `geometry.bounds_for_subtree`
    is the harness supervisor's own world-space AABB walker — the same numbers an
    agent gets from `GET /scene/tree?bounds=1` — so the watchdog and the agent
    agree. Same budget rule as the walk above; a node whose bounds we cannot
    afford falls back to its origin z.
    """
    helpers = os.path.join(os.environ.get("OMNISIM_HOME", ""), "projects",
                           "default", "controllers", "harness_supervisor")
    if os.path.isdir(helpers) and helpers not in sys.path:
        sys.path.insert(0, helpers)
    try:
        import geometry  # noqa: PLC0415
    except Exception:  # noqa: BLE001
        geometry = None

    floor = None
    truncated = False
    for node in static_nodes:
        z = None
        if geometry is not None and time.monotonic() <= deadline:
            try:
                b = geometry.bounds_for_subtree(node)
                if b:
                    z = float(b["bbox_min"][2])
            except Exception:  # noqa: BLE001
                z = None
        elif geometry is not None:
            truncated = True
        if z is None:
            try:
                z = float(node.getPosition()[2])
            except Exception:  # noqa: BLE001
                continue
        floor = z if floor is None else min(floor, z)
    return floor, truncated


def main() -> int:
    args = parse_args(sys.argv[1:])
    out_path = args.get("out") or os.environ.get("OMNISIM_RUNAWAY_OUT") \
        or "runaway_watchdog.jsonl"
    period = max(1, int(float(args.get("period-steps", 8))))
    max_samples = max(1, int(float(args.get("max-samples", 20000))))

    budget = max(0.5, float(args.get("scan-budget-s", 4.0)))

    supervisor = Supervisor()
    dt = int(supervisor.getBasicTimeStep())
    tracked, skipped = _top_level_bodies(supervisor)
    dynamic = [node for node, _container in tracked]
    containers = [container for _node, container in tracked]

    def key_of(node, index):
        return _def_of(node) or _sf_string(node, "name") or f"#{index}"

    keys = [key_of(n, i) for i, n in enumerate(dynamic)]
    fh = open(out_path, "w", buffering=1, encoding="utf-8")

    def write(obj):
        fh.write(json.dumps(obj) + "\n")
        fh.flush()

    # The header goes out BEFORE the (bounded, but not free) static-collider
    # scan, so a run that is killed during the scan still leaves evidence of
    # what the watchdog saw instead of an empty file.
    write({
        "kind": "header",
        "dt_ms": dt,
        "period_steps": period,
        "static_floor_z": None,
        "static_bodies": 0,
        "bodies": [
            {"key": keys[i], "name": _sf_string(n, "name"), "type": _typename(n),
             # `via` names the Group/Pose this body was found under, so a
             # nested roster is not mistaken for a top-level one.
             "via": (_def_of(containers[i]) or _typename(containers[i]))
                    if containers[i] is not None else None}
            for i, n in enumerate(dynamic)
        ],
        # What was walked and NOT tracked. An empty `bodies` list is a FAIL in
        # the runner (a roster of zero cannot certify anything), and this is
        # what makes that FAIL actionable instead of just loud.
        "skipped": skipped[:24],
        "skipped_total": len(skipped),
    })

    scan_t0 = time.monotonic()
    static, visits, walk_truncated = _static_colliders(supervisor, scan_t0 + budget * 0.5)
    floor_z, bounds_truncated = _floor_reference(static, scan_t0 + budget)
    write({
        "kind": "floor",
        "static_floor_z": floor_z,
        "static_bodies": len(static),
        "nodes_visited": visits,
        "truncated": bool(walk_truncated or bounds_truncated),
        "scan_ms": int((time.monotonic() - scan_t0) * 1000),
    })

    samples = 0
    while samples < max_samples:
        if supervisor.step(dt * period) == -1:
            break
        row = {}
        for i, node in enumerate(dynamic):
            try:
                p = node.getPosition()
                row[keys[i]] = [float(p[0]), float(p[1]), float(p[2])]
            except Exception:  # noqa: BLE001
                continue
        samples += 1
        write({"kind": "s", "t": float(supervisor.getTime()), "z": row})
    write({"kind": "end", "t": float(supervisor.getTime()), "samples": samples})
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:  # noqa: BLE001
        # A diagnostic that crashes the run it is diagnosing is worse than no
        # diagnostic: report on stderr and exit 0 so the runner's
        # controller-failure patterns do not fire on the watchdog itself. A
        # missing/short JSONL is what the runner reports instead.
        sys.stderr.write("[runaway_watchdog] sampling aborted:\n"
                         + traceback.format_exc())
        sys.exit(0)
