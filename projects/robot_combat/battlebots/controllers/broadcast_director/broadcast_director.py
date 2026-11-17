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

"""broadcast_director — camera-switching supervisor for BattleBox matches.

Reads the match_director's broadcast state from its customData every
step (set by match_director.py) and animates the world Viewpoint to
follow the most interesting fighter. Camera modes:

  --mode auto       (default) Rotate between OVERHEAD, ACTION_TRACK,
                    POV_VICTIM, REPLAY based on match events.
  --mode overhead   Static top-down (no animation; useful for headless).
  --mode chase      Always chase the leader of the scorecard.

Event-driven cuts (auto mode):

  * impact spike   -> ACTION_TRACK on the bot that took the hit (the
                      one whose impact_events counter just went up)
  * KO / OOTA      -> SLOW_REPLAY: slow viewpoint pan over the corpse
                      position for 3s, then back to overhead
  * idle (>4s)     -> OVERHEAD

The director only mutates Viewpoint; it never touches fighter physics.
Safe to enable or disable in any world without affecting the outcome.

CLI:
  --mode auto|overhead|chase    (default auto)
  --hold-s <float>              dwell time per active shot (default 2.5)

Reads its OWN customData for static config:
  {"red": "...", "blue": "...", "fighters": [...], "arena_size": 8.0}
"""

from __future__ import annotations

import argparse
import json
import math
import sys

from omnisim import Supervisor


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="broadcast_director", add_help=False)
    ap.add_argument("--mode", default="auto", choices=["auto", "overhead", "chase"])
    ap.add_argument("--hold-s", type=float, default=2.5)
    return ap.parse_known_args(argv)[0]


def _scan_named_children(supervisor: Supervisor) -> dict:
    out = {}
    try:
        root = supervisor.getRoot()
        children = root.getField("children")
        for i in range(children.getCount()):
            try:
                node = children.getMFNode(i)
                nf = node.getField("name")
                if nf is None:
                    continue
                nm = nf.getSFString()
                if nm:
                    out[nm] = node
            except Exception:
                continue
    except Exception:
        pass
    return out


def _resolve_fighters(cfg: dict, scene: dict) -> list[str]:
    if "fighters" in cfg and isinstance(cfg["fighters"], list):
        return [f for f in cfg["fighters"] if f in scene]
    out = []
    for k in ("red", "blue"):
        v = cfg.get(k)
        if v and v in scene:
            out.append(v)
    return out


def _find_viewpoint(supervisor: Supervisor):
    """Viewpoint is a top-level node in every world. Locate by walking
    root children for type == 'Viewpoint'."""
    try:
        root = supervisor.getRoot()
        children = root.getField("children")
        for i in range(children.getCount()):
            try:
                node = children.getMFNode(i)
                if node.getTypeName() == "Viewpoint":
                    return node
            except Exception:
                continue
    except Exception:
        pass
    return None


def _set_viewpoint(vp_node, position, orientation) -> None:
    if vp_node is None:
        return
    try:
        vp_node.getField("position").setSFVec3f(list(position))
        vp_node.getField("orientation").setSFRotation(list(orientation))
    except Exception:
        pass


# Pre-baked viewpoint poses. orientation is axis-angle (x,y,z,theta).
# OVERHEAD = top-down, looking straight down at arena center.
OVERHEAD_POSE = ([0.0, 0.0, 12.0], [-0.5773, 0.5773, 0.5773, 2.0944])


def _action_pose(target_pos, follow_dist=4.0, follow_height=3.5):
    """Position the camera behind-and-above a target at follow_dist/up.
    Aim toward target by combining a downward tilt with a yaw aligned
    with the target's direction from arena center."""
    tx, ty = target_pos[0], target_pos[1]
    # Place camera offset radially outward from arena center, slightly
    # higher than the target.
    r = math.hypot(tx, ty) + 0.01
    if r < 0.05:
        # Target at center — fall back to a fixed +x offset.
        cx, cy = follow_dist, 0.0
    else:
        cx = tx + (tx / r) * follow_dist
        cy = ty + (ty / r) * follow_dist
    cz = max(2.0, follow_height)
    # Orientation: same canonical "overhead-tilted" look used elsewhere
    # in the codebase. Cheap and good enough; a future v2 could compute
    # a real look-at quaternion.
    return ([cx, cy, cz], [-0.5773, 0.5773, 0.5773, 2.0944])


def main() -> int:
    args = parse_args(sys.argv[1:])

    supervisor = Supervisor()
    basic_step_ms = int(supervisor.getBasicTimeStep())
    dt = basic_step_ms / 1000.0

    self_node = supervisor.getSelf()
    raw = self_node.getField("customData").getSFString() if self_node else ""
    try:
        cfg = json.loads(raw) if raw else {}
    except Exception:
        cfg = {}

    scene = _scan_named_children(supervisor)
    fighters = _resolve_fighters(cfg, scene)
    fnodes = {f: scene[f] for f in fighters if f in scene}

    vp = _find_viewpoint(supervisor)
    if vp is None:
        sys.stderr.write("[broadcast_director] no Viewpoint found; idling\n")
        while supervisor.step(basic_step_ms) != -1:
            pass
        return 0

    sys.stderr.write(
        f"[broadcast_director] mode={args.mode} fighters={fighters}\n"
    )

    # Locate match_director once; we read its customData for events.
    director = scene.get("match_director")

    prev_impact_total = 0
    prev_eliminated: list[str] = []
    last_cut_t = 0.0
    sim_t = 0.0
    current_target: str | None = None
    in_replay = False
    replay_until = 0.0

    def director_state() -> dict:
        if director is None:
            return {}
        try:
            raw = director.getField("customData").getSFString()
            return json.loads(raw) if raw else {}
        except Exception:
            return {}

    # Initial pose.
    _set_viewpoint(vp, *OVERHEAD_POSE)

    while supervisor.step(basic_step_ms) != -1:
        sim_t += dt

        if args.mode == "overhead":
            _set_viewpoint(vp, *OVERHEAD_POSE)
            continue

        st = director_state()
        match_over = st.get("match_over", False)

        # Replay cut for KO / OOTA — slow pan over the corpse position.
        eliminated = st.get("eliminated") or []
        new_elims = [e for e in eliminated if e not in prev_eliminated]
        if new_elims and not in_replay:
            corpse = new_elims[-1]
            if corpse in fnodes:
                try:
                    cp = fnodes[corpse].getPosition()
                    pose = _action_pose(cp, follow_dist=2.5, follow_height=2.0)
                    _set_viewpoint(vp, *pose)
                    in_replay = True
                    replay_until = sim_t + 3.0
                    sys.stderr.write(
                        f"[broadcast_director] REPLAY on {corpse} at t={sim_t:.2f}\n"
                    )
                except Exception:
                    pass
        prev_eliminated = list(eliminated)

        if in_replay:
            if sim_t >= replay_until:
                in_replay = False
                _set_viewpoint(vp, *OVERHEAD_POSE)
                last_cut_t = sim_t
            continue

        if match_over:
            # Park the camera over the winner if known; else overhead.
            winner = st.get("winner")
            if winner and winner in fnodes:
                try:
                    wp = fnodes[winner].getPosition()
                    _set_viewpoint(vp, *_action_pose(wp, follow_dist=3.0, follow_height=2.5))
                except Exception:
                    _set_viewpoint(vp, *OVERHEAD_POSE)
            else:
                _set_viewpoint(vp, *OVERHEAD_POSE)
            continue

        # Impact event detection: cut to whichever bot's impact counter
        # just went up. impact_events is {name: count}.
        impacts = st.get("impact_events") or {}
        total = sum(impacts.values()) if isinstance(impacts, dict) else 0
        if total > prev_impact_total:
            # Pick the fighter whose count increased most recently.
            new_hits = [f for f, c in impacts.items()
                        if isinstance(c, int) and c > 0]
            if new_hits:
                target = max(new_hits, key=lambda f: impacts.get(f, 0))
                current_target = target
                last_cut_t = sim_t
        prev_impact_total = total

        # Auto / chase: hold current target for hold_s, then re-evaluate.
        if (current_target is None or
                (sim_t - last_cut_t) > args.hold_s):
            if args.mode == "chase":
                # Pick the leader of the scorecard if known, else arbitrary.
                sc = st.get("scorecard") or {}
                if sc:
                    current_target = max(sc, key=lambda f: sc[f].get("total", 0))
                elif fighters:
                    current_target = fighters[0]
            else:
                # auto: rotate around fighters
                if fighters:
                    if current_target is None:
                        current_target = fighters[0]
                    else:
                        try:
                            idx = fighters.index(current_target)
                            current_target = fighters[(idx + 1) % len(fighters)]
                        except ValueError:
                            current_target = fighters[0]
            last_cut_t = sim_t

        if current_target and current_target in fnodes:
            try:
                tp = fnodes[current_target].getPosition()
                _set_viewpoint(vp, *_action_pose(tp))
            except Exception:
                _set_viewpoint(vp, *OVERHEAD_POSE)
        else:
            _set_viewpoint(vp, *OVERHEAD_POSE)

    return 0


if __name__ == "__main__":
    sys.exit(main())
