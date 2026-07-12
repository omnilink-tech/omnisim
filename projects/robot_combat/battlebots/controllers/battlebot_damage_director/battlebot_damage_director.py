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

"""battlebot_damage_director — physics-based damage + match termination.

Replaces the judge-scoring `match_director` for BattleBot duels. There
is no scorecard: the winner is the bot still moving when the other can
no longer move.

Per fighter the director tracks per-part HP:

  - chassis                                            (body health)
  - front_left_wheel, front_right_wheel,
    rear_left_wheel, rear_right_wheel                  (4 wheel HP)
  - weapon_disc / weapon_hammer_arm / weapon_flipper_arm  (if present)

Each step, every tracked body's velocity-change magnitude × body mass
gives an impulse estimate. When the two fighters are in proximity
(gap < `contact_gate_m`), an impulse on a body above its damage
threshold deducts HP from that part. Self-induced jolts (motor spin-up,
brain REVERSE commands) don't trigger damage because they don't
co-occur with proximity.

When a wheel reaches HP=0:
  - The HingeJoint owning it is removed from the chassis. The wheel is
    re-spawned as a free Solid (cylinder, same mass/geometry) at the
    wheel's last world pose, inheriting its last velocity — the wheel
    visibly falls off and rolls away.
  - The chassis now has one fewer point of contact + traction.

When the weapon body reaches HP=0:
  - Same detach treatment: weapon disc / arm flies free.

Match end:
  - A bot with `fail_wheels` (default 3) or more lost wheels can no
    longer drive → declared immobilized.
  - A bot OOTA (outside arena footprint by margin) → immobilized.
  - First to be immobilized loses; if both in same step → draw.

Writes a final JSON summary to `output` path (default scratch dir).

customData schema (on this supervisor node):
  {
    "fighters":   ["red", "blue"],
    "arena_size": 8.0,
    "fail_wheels": 3,
    "contact_gate_m": 1.2,
    "output":     "_scratch/battlebox_match.json"
  }

The director also mirrors its current state back into its own
customData every step so the BattleBot brain can read it and stop
driving once `match_over` flips true:
  { "match_over": bool, "winner": str|null, "broken": {bot: [parts]} }
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

from omnisim import Supervisor

WHEEL_DEFS = (
    "FL_WHEEL",
    "FR_WHEEL",
    "RL_WHEEL",
    "RR_WHEEL",
)
# Wedge / spinner / hammer / flipper all DEF their main body as
# WEAPON_BODY inside BattleBot.proto.
WEAPON_DEF = "WEAPON_BODY"

DEFAULTS = {
    "fighters": [],
    "arena_size": 8.0,
    "fail_wheels": 3,
    "contact_gate_m": 1.2,
    "oota_margin": 0.15,
    "timer_s": 240.0,     # match cap — usually triggered before this
    "output": "_scratch/battlebox_match.json",
    # Per-part HP and impulse threshold. impulse = mass * |Δv|; only
    # impulse above threshold actually subtracts HP, so steady rolling
    # contact (low impulse) doesn't slowly grind parts to dust.
    "hp": {
        "chassis": 150.0,
        "wheel": 35.0,
        "weapon": 60.0,
    },
    "impulse_threshold_J": {
        "chassis": 1.5,
        "wheel": 0.8,
        "weapon": 1.0,
    },
}


def _xy_dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _vmag(v) -> float:
    return math.sqrt(sum(c * c for c in v))


def _resolve_scratch_output(raw):
    """Anchor the match-result path under a real scratch dir.

    The DEFAULTS / world customData historically hardcoded ``_scratch`` (a dev's
    old drive). On any other machine that silently writes the scorecard to a missing drive, so
    headless validation has no oracle. Re-anchor to ``<OMNISIM_HOME>/_scratch/<name>`` whenever
    the configured path is relative or its parent directory does not exist; honor a configured
    absolute path whose parent already exists. Falls back to the repo root derived from this
    file's location so it works even when OMNISIM_HOME is unset in the controller environment.
    """
    try:
        p = Path(raw)
        if p.is_absolute() and p.parent.exists():
            return str(p)
        name = p.name or "battlebox_match.json"
    except Exception:
        name = "battlebox_match.json"
    # NB: do NOT fall back to WEBOTS_HOME -- on dev machines it points at the *system* Webots
    # install (C:\Program Files\Webots), not this repo. The __file__-derived repo root is the
    # bulletproof, env-independent anchor.
    home = os.environ.get("OMNISIM_HOME") or str(Path(__file__).resolve().parents[5])  # repo root
    return str(Path(home) / "_scratch" / name)


def _collect_descendant_solids(root):
    """DFS walk a node subtree, yielding every Solid/Robot encountered.
    Works for both authored Robots and PROTO instances because the
    Solid hierarchy is exposed at the scene-graph level."""
    out = []
    if root is None:
        return out
    stack = [root]
    seen = set()
    while stack:
        n = stack.pop()
        try:
            nid = n.getId()
        except Exception:
            nid = id(n)
        if nid in seen:
            continue
        seen.add(nid)
        try:
            t = n.getTypeName()
        except Exception:
            t = ""
        if t in ("Solid", "Robot", "URDFRobot", "BattleBot"):
            out.append(n)
        # Walk children + endPoint (HingeJoint wraps a Solid in endPoint).
        for fname in ("children", "endPoint", "bodySlot"):
            try:
                f = n.getField(fname)
            except Exception:
                f = None
            if f is None:
                continue
            try:
                if fname in ("endPoint", "bodySlot"):
                    child = f.getSFNode()
                    if child is not None:
                        stack.append(child)
                else:
                    cnt = f.getCount()
                    for i in range(cnt):
                        ch = f.getMFNode(i)
                        if ch is not None:
                            stack.append(ch)
            except Exception:
                continue
    return out


def _find_descendant_solid(node, target_name):
    """Find first Solid under `node` whose name == target_name."""
    for s in _collect_descendant_solids(node):
        try:
            nm = s.getField("name").getSFString()
        except Exception:
            continue
        if nm == target_name:
            return s
    return None


def _get_mass(node) -> float:
    """Read `physics.mass` from a Solid; fall back to 1.0 kg."""
    try:
        phys = node.getField("physics").getSFNode()
        if phys is None:
            return 1.0
        m = phys.getField("mass").getSFFloat()
        return float(m) if m and m > 0 else 1.0
    except Exception:
        return 1.0


def _replace_first_line(text: str, field_name: str, new_line: str):
    """Replace the first VRML line whose first non-whitespace token is
    `field_name`. Returns (new_text, n_replaced)."""
    lines = text.split("\n")
    for i, ln in enumerate(lines):
        stripped = ln.lstrip()
        if stripped.startswith(field_name + " ") or stripped == field_name:
            # Preserve leading whitespace from the original.
            lead = ln[:len(ln) - len(stripped)]
            lines[i] = lead + new_line
            return "\n".join(lines), 1
    return text, 0


def _aa_from_matrix(R):
    """Webots 9-elt row-major rotation matrix -> axis-angle (4 floats).
    Robust for small angles."""
    # Trace -> angle. Clamp for numerical safety.
    tr = R[0] + R[4] + R[8]
    cos_t = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    angle = math.acos(cos_t)
    if angle < 1e-6:
        return (0.0, 0.0, 1.0, 0.0)
    s = 2.0 * math.sin(angle)
    ax = (R[7] - R[5]) / s
    ay = (R[2] - R[6]) / s
    az = (R[3] - R[1]) / s
    return (ax, ay, az, angle)


def _nearest_part(fighter, pt):
    """Find the non-detached part of `fighter` whose centroid is
    closest to world point `pt`. Returns part_id or None."""
    best = None
    best_d2 = float("inf")
    for part_id, p in fighter.parts.items():
        if p["detached"]:
            continue
        try:
            pp = p["node"].getPosition()
        except Exception:
            continue
        dx = pp[0] - pt[0]
        dy = pp[1] - pt[1]
        dz = pp[2] - pt[2]
        d2 = dx * dx + dy * dy + dz * dz
        if d2 < best_d2:
            best_d2 = d2
            best = part_id
    return best


def _collect_node_ids(root):
    """DFS through a bot's scene subtree, collecting every node's
    integer id. Used to recognise which Webots body in a ContactPoint
    belongs to which fighter."""
    out = set()
    if root is None:
        return out
    stack = [root]
    seen = set()
    while stack:
        n = stack.pop()
        try:
            nid = n.getId()
        except Exception:
            continue
        if nid in seen:
            continue
        seen.add(nid)
        out.add(nid)
        for fname in ("children", "endPoint"):
            try:
                f = n.getField(fname)
            except Exception:
                f = None
            if f is None:
                continue
            try:
                if fname == "endPoint":
                    ch = f.getSFNode()
                    if ch is not None:
                        stack.append(ch)
                else:
                    for i in range(f.getCount()):
                        ch = f.getMFNode(i)
                        if ch is not None:
                            stack.append(ch)
            except Exception:
                continue
    return out


class Fighter:
    def __init__(self, name, node):
        self.name = name
        self.node = node
        self.parts = {}   # part_id -> {"node":, "mass":, "hp":, "max":, "prev_v":, "detached": bool}
        self.broken = []  # list of part_ids that have detached
        self.immobile = False
        self.disqualify_reason = None
        self.own_node_ids = set()  # populated after part discovery
        # node-id -> part_id mapping, so we can look up a ContactPoint
        # whose `node_id` refers to one of our own bodies.
        self.id_to_part = {}

    def add_part(self, part_id, node, hp_max):
        if node is None:
            return
        mass = _get_mass(node)
        try:
            v0 = list(node.getVelocity())[:3]
        except Exception:
            v0 = [0.0, 0.0, 0.0]
        self.parts[part_id] = {
            "node": node, "mass": mass, "hp": hp_max, "max": hp_max,
            "prev_v": v0, "detached": False,
            # last_damage_t: time of the most recent damage tick on this
            # part. We rate-limit consecutive damage so a single contact
            # event (which spans many physics steps) only deals ONE big
            # hit, not five small ones in 30 ms.
            "last_damage_t": -1.0,
        }
        try:
            self.id_to_part[node.getId()] = part_id
        except Exception:
            pass


def main():
    sup = Supervisor()
    step_ms = int(sup.getBasicTimeStep())
    dt = step_ms / 1000.0

    # Config.
    cfg = dict(DEFAULTS)
    try:
        raw = sup.getCustomData()
        if raw:
            cfg.update(json.loads(raw))
    except Exception as exc:
        sys.stderr.write(f"[damage_director] customData parse failed: {exc}\n")

    # Headless-validation override: bound the match clock without editing customData.
    cfg["timer_s"] = float(os.environ.get("OMNISIM_DAMAGE_TIMER_S", cfg["timer_s"]))

    fighter_names = cfg["fighters"]
    if not fighter_names:
        # Auto-detect: scan top-level Robot nodes for BattleBots.
        try:
            children = sup.getRoot().getField("children")
            for i in range(children.getCount()):
                ch = children.getMFNode(i)
                try:
                    nm = ch.getField("name").getSFString()
                    ty = ch.getTypeName()
                    if (nm not in ("match_director", "broadcast_director",
                                   "duel_tracker", "damage_director",
                                   "sun_marker")
                            and ty == "Robot"):
                        fighter_names.append(nm)
                except Exception:
                    continue
        except Exception:
            pass

    if len(fighter_names) < 1:
        sys.stderr.write("[damage_director] no fighters detected; idling\n")
        while sup.step(step_ms) != -1:
            pass
        return 0

    sys.stderr.write(f"[damage_director] tracking {fighter_names}\n")

    # Build per-fighter part registries.
    fighters = []
    for nm in fighter_names:
        # Find the bot's Robot node by walking root children.
        bot_node = None
        try:
            rc = sup.getRoot().getField("children")
            for i in range(rc.getCount()):
                ch = rc.getMFNode(i)
                try:
                    if ch.getField("name").getSFString() == nm:
                        bot_node = ch
                        break
                except Exception:
                    continue
        except Exception:
            pass
        if bot_node is None:
            sys.stderr.write(f"[damage_director] {nm}: bot node not found\n")
            continue
        f = Fighter(nm, bot_node)
        # Chassis = the Robot itself.
        f.add_part("chassis", bot_node, cfg["hp"]["chassis"])
        # Parts are at world scope now (the duel world inlines each
        # bot as a Robot{} so Supervisor.parent.remove() actually
        # works). DEFs are namespaced by uppercased bot name:
        # RED_FL_WHEEL, BLUE_RR_WHEEL, RED_WEAPON, ...
        prefix = nm.upper()
        for wdef in WHEEL_DEFS:
            full_def = f"{prefix}_{wdef}"
            try:
                wnode = sup.getFromDef(full_def)
            except Exception:
                wnode = None
            part_id = wdef.lower()
            f.add_part(part_id, wnode, cfg["hp"]["wheel"])
        try:
            weap = sup.getFromDef(f"{prefix}_WEAPON")
        except Exception:
            weap = None
        if weap is not None:
            f.add_part("weapon", weap, cfg["hp"]["weapon"])
        sys.stderr.write(
            f"[damage_director] {nm}: parts={list(f.parts.keys())}\n"
        )
        fighters.append(f)

    # Contact-point discovery: enable per-fighter tracking so we can
    # ask each step "which OTHER body am I currently touching?". The
    # NodeId returned in each ContactPoint lets us tell apart
    # bot-on-bot contacts from bot-on-floor.
    for f in fighters:
        f.own_node_ids = _collect_node_ids(f.node)
        try:
            f.node.enableContactPointsTracking(step_ms, True)
        except Exception as exc:
            sys.stderr.write(
                f"[damage_director] {f.name}: enableContactPointsTracking "
                f"failed: {exc}\n"
            )
        sys.stderr.write(
            f"[damage_director] {f.name}: id_to_part = "
            f"{f.id_to_part}\n"
        )

    half = cfg["arena_size"] * 0.5
    margin = cfg["oota_margin"]
    gate = cfg["contact_gate_m"]
    fail_wheels = int(cfg["fail_wheels"])
    th = cfg["impulse_threshold_J"]

    # L4: opt-in real-penetration damage magnitude (mirrors damage_tracker).
    # OMNISIM_DAMAGE_USE_DEPTH=1 scores a part's strike on the engine's real
    # per-contact penetration cp.depth instead of the mass*|Δv| momentum proxy
    # — backend-symmetric, free of Newton's per-step body_qd jitter. Default
    # OFF → scoring byte-unchanged. Wheels (load-bearing) keep the proxy; depth
    # scores chassis/weapon strikes. The bot-on-bot contact gate already
    # excludes resting ground contacts, so the damage_tracker wheel-flood is not
    # a concern here — wheels are excluded only for policy consistency. See
    # physics-contact-impulse-api.md §7.
    use_depth = os.environ.get(
        "OMNISIM_DAMAGE_USE_DEPTH", "0") not in ("", "0", "false", "False")
    try:
        depth_scale = float(os.environ.get("OMNISIM_DAMAGE_DEPTH_SCALE", "100.0"))
    except ValueError:
        depth_scale = 100.0
    if depth_scale <= 0.0:
        depth_scale = 100.0
    if use_depth:
        sys.stderr.write(
            f"[damage_director] depth scoring ON (scale={depth_scale:g}); "
            f"wheels stay on the velocity proxy\n")

    root_children = sup.getRoot().getField("children")
    detached_counter = 0

    def detach_part(f, part_id):
        """Detach the part from its bot. We can't directly reparent a
        node in Webots, so we (1) export the part's VRML so we keep
        the actual geometry / appearance / physics, (2) rewrite its
        top-level translation+rotation to world coordinates, (3) import
        as a free root-child Solid, (4) transfer its velocity, (5)
        remove the original HingeJoint from the chassis. Visually this
        looks like the original wheel/disc coming off — the new node
        is geometrically identical because it's literally the same
        VRML, just reparented to the world root."""
        nonlocal detached_counter
        p = f.parts.get(part_id)
        if p is None or p["detached"]:
            return
        wnode = p["node"]
        try:
            pos = list(wnode.getPosition())
            ori = list(wnode.getOrientation())
            vel = list(wnode.getVelocity())[:3]
        except Exception as exc:
            sys.stderr.write(
                f"[damage_director] detach {f.name}.{part_id}: pose read "
                f"failed: {exc}\n"
            )
            return
        aa = _aa_from_matrix(ori)

        # Chassis "death" isn't a detach — the bot stays in place, we
        # just mark it immobile. Removing the Robot would crash the
        # brain mid-step.
        if part_id == "chassis":
            f.broken.append(part_id)
            p["detached"] = True
            return

        try:
            exported = wnode.exportString()
        except Exception as exc:
            sys.stderr.write(
                f"[damage_director] detach {f.name}.{part_id}: exportString "
                f"failed: {exc}\n"
            )
            exported = None

        detached_counter += 1
        free_name = f"DETACHED_{f.name}_{part_id}_{detached_counter}"

        stanza = None
        if exported:
            # Rewrite the top-level Solid's translation + rotation to
            # WORLD coordinates (exportString gives local), and prepend
            # a DEF so we can look it up.
            new_translation = f"translation {pos[0]} {pos[1]} {pos[2]}"
            new_rotation = (
                f"rotation {aa[0]} {aa[1]} {aa[2]} {aa[3]}"
            )
            patched, n_tr = _replace_first_line(exported, "translation",
                                                new_translation)
            if n_tr == 0:
                # No translation field present — inject one right after
                # the opening brace.
                patched = patched.replace("Solid {",
                                          f"Solid {{\n  {new_translation}",
                                          1)
            patched, _ = _replace_first_line(patched, "rotation",
                                             new_rotation)
            if "rotation " not in patched.split("Solid {", 1)[-1][:200]:
                patched = patched.replace("Solid {",
                                          f"Solid {{\n  {new_rotation}",
                                          1)
            # OmniSim requires unique top-level names; rename so it
            # doesn't collide with the chassis-side wheel that we're
            # about to remove.
            patched, _ = _replace_first_line(patched, "name",
                                             f'name "{free_name}"')
            # Strip any leading `DEF <name> ` that exportString() may
            # have emitted on the outer Solid (when the original part
            # had a DEF in the world file). Without this, the next
            # line produces `DEF <old> DEF <new> Solid { ... }` which
            # the parser rejects as "Expected node or PROTO name,
            # found 'DEF'." — non-fatal but it logs an ugly warning.
            import re as _re
            patched = _re.sub(r'^\s*DEF\s+\S+\s+(Solid\s*\{)',
                              r'\1', patched, count=1)
            # Add our own DEF on the outer Solid so getFromDef finds it.
            patched = patched.replace("Solid {", f"DEF {free_name} Solid {{", 1)
            stanza = patched

        if stanza is None:
            # Fallback path: spawn a generic cylinder. Should not be
            # reached in practice, but keep the behaviour safe.
            if part_id.endswith("wheel"):
                h, r, mass = 0.05, 0.09, 0.6
            else:
                h, r, mass = 0.04, 0.15, 2.0
            stanza = (
                f'DEF {free_name} Solid {{'
                f'  translation {pos[0]} {pos[1]} {pos[2]}'
                f'  rotation {aa[0]} {aa[1]} {aa[2]} {aa[3]}'
                f'  name "{free_name}"'
                f'  children [ Shape {{'
                f'    appearance PBRAppearance {{ baseColor 0.4 0.4 0.4 '
                f'metalness 0.7 roughness 0.4 }}'
                f'    geometry Cylinder {{ height {h} radius {r} }}'
                f'  }} ]'
                f'  boundingObject Cylinder {{ height {h} radius {r} }}'
                f'  physics Physics {{ density -1 mass {mass} }}'
                f'}}'
            )

        try:
            root_children.importMFNodeFromString(-1, stanza)
        except Exception as exc:
            sys.stderr.write(
                f"[damage_director] detach {f.name}.{part_id}: spawn failed: "
                f"{exc}\n"
            )
            return

        new_node = sup.getFromDef(free_name)
        if new_node is not None:
            try:
                new_node.setVelocity(vel + [0.0, 0.0, 0.0])
            except Exception:
                pass

        # Remove the HingeJoint that owned this Solid.
        try:
            parent = wnode.getParentNode()
        except Exception as exc:
            sys.stderr.write(
                f"[damage_director] detach {f.name}.{part_id}: getParentNode "
                f"failed: {exc}\n"
            )
            parent = None
        removed_parent = False
        if parent is not None:
            try:
                parent.remove()
                removed_parent = True
            except Exception as exc:
                sys.stderr.write(
                    f"[damage_director] detach {f.name}.{part_id}: "
                    f"parent.remove failed: {exc}\n"
                )

        f.broken.append(part_id)
        p["detached"] = True

        # Numerical post-condition check: the world-scope DEF for
        # this part should no longer resolve after the parent
        # HingeJoint is removed.
        verification = "skipped"
        prefix = f.name.upper()
        world_def = None
        if part_id.endswith("wheel"):
            world_def = {
                "fl_wheel": f"{prefix}_FL_WHEEL",
                "fr_wheel": f"{prefix}_FR_WHEEL",
                "rl_wheel": f"{prefix}_RL_WHEEL",
                "rr_wheel": f"{prefix}_RR_WHEEL",
            }.get(part_id)
        elif part_id == "weapon":
            world_def = f"{prefix}_WEAPON"
        if world_def is not None:
            try:
                still_there = sup.getFromDef(world_def)
                verification = "GONE" if still_there is None else "STILL_PRESENT"
            except Exception:
                verification = "GONE (lookup raised)"

        sys.stderr.write(
            f"[damage_director] {f.name}.{part_id} detached at "
            f"({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) -> {free_name} "
            f"(parent_removed={removed_parent}, in-bot={verification})\n"
        )

    # Validation hook (default off): force-detach parts regardless of HP, to exercise the
    # Newton node-graph mutation (exportString -> importMFNodeFromString -> parent.remove) in
    # isolation from collision/damage. Format: "fl_wheel,weapon" (parts of fighters[0]) or
    # "red:fl_wheel,blue:weapon" (explicit fighter:part). Fires once at
    # OMNISIM_DAMAGE_FORCE_DETACH_T seconds (default 2.0).
    _force_detach = []
    _fd_raw = os.environ.get("OMNISIM_DAMAGE_FORCE_DETACH", "").strip()
    if _fd_raw:
        _fd_by_name = {f.name: f for f in fighters}
        for _tok in _fd_raw.split(","):
            _tok = _tok.strip()
            if not _tok:
                continue
            if ":" in _tok:
                _who, _part = _tok.split(":", 1)
                _tgt = _fd_by_name.get(_who.strip())
            else:
                _part = _tok
                _tgt = fighters[0] if fighters else None
            if _tgt is not None:
                _force_detach.append((_tgt, _part.strip()))
    _fd_t = float(os.environ.get("OMNISIM_DAMAGE_FORCE_DETACH_T", "2.0"))
    _fd_done = False

    sim_t = 0.0
    match_over = False
    winner = None
    win_reason = None
    end_writeout_done = False

    def write_state():
        st = {
            "match_over": match_over,
            "winner": winner,
            "broken": {f.name: list(f.broken) for f in fighters},
            "eliminated": [f.name for f in fighters if f.immobile],
            "t_s": round(sim_t, 2),
        }
        try:
            sup.setCustomData(json.dumps(st))
        except Exception:
            pass

    while sup.step(step_ms) != -1:
        sim_t += dt

        if _force_detach and not _fd_done and sim_t >= _fd_t:
            _fd_done = True
            for _tgt, _part in _force_detach:
                sys.stderr.write(
                    f"[damage_director] FORCE-DETACH {_tgt.name}.{_part} "
                    f"at t={sim_t:.2f}s\n")
                detach_part(_tgt, _part)

        if not match_over:
            # Gather each fighter's contact points (a ContactPoint's
            # node_id refers to the fighter's OWN body — i.e. which of
            # ITS Solids has the contact). To find bot-on-bot strikes
            # we cross-reference: a contact point on RED at world
            # position P that also appears in BLUE's list (within
            # tolerance) means RED's body cp.node_id is touching BLUE's
            # body whose corresponding cp also reports P.
            cps_by_name = {}
            pos_by_name = {}
            for f in fighters:
                try:
                    cps_by_name[f.name] = f.node.getContactPoints(True)
                except Exception:
                    cps_by_name[f.name] = []
                try:
                    pos_by_name[f.name] = f.node.getPosition()
                except Exception:
                    pos_by_name[f.name] = None

            parts_in_contact = {f.name: set() for f in fighters}
            # Max bot-on-bot penetration depth seen per part this step, for the
            # opt-in depth scoring below. Captured here because this is where
            # the matched ContactPoints (and their cp.depth) are in scope.
            depth_in_contact = {f.name: {} for f in fighters}
            POS_TOL2 = 0.10 * 0.10  # 10 cm tolerance for matching
            # Broad-phase prune (P5): each bot's contact list includes its own
            # wheel-ground contacts, so without a gate every fighter pair runs the
            # full cp x cp cross-reference even when the bots are metres apart. Two
            # bots can only share a contact point if their chassis centres are close,
            # so skip far-apart pairs -- turns O(F^2 * C^2) toward O(neighbours) in a
            # spread-out melee (matters most in the 20-40 m ORC arenas).
            prune_m = cfg["contact_gate_m"] + 2.0
            prune_m2 = prune_m * prune_m
            for i, fi in enumerate(fighters):
                pi_pos = pos_by_name.get(fi.name)
                for j in range(i + 1, len(fighters)):
                    fj = fighters[j]
                    pj_pos = pos_by_name.get(fj.name)
                    if pi_pos is not None and pj_pos is not None:
                        ddx = pi_pos[0] - pj_pos[0]
                        ddy = pi_pos[1] - pj_pos[1]
                        if ddx * ddx + ddy * ddy > prune_m2:
                            continue  # far apart -- no shared contact possible
                    for cp_i in cps_by_name[fi.name]:
                        for cp_j in cps_by_name[fj.name]:
                            dx = cp_i.point[0] - cp_j.point[0]
                            dy = cp_i.point[1] - cp_j.point[1]
                            dz = cp_i.point[2] - cp_j.point[2]
                            if dx * dx + dy * dy + dz * dz < POS_TOL2:
                                pi = fi.id_to_part.get(cp_i.node_id)
                                pj = fj.id_to_part.get(cp_j.node_id)
                                if pi is None:
                                    # Spatial fallback: assign to the
                                    # nearest non-detached part.
                                    pi = _nearest_part(fi, cp_i.point)
                                if pj is None:
                                    pj = _nearest_part(fj, cp_j.point)
                                if pi:
                                    parts_in_contact[fi.name].add(pi)
                                    di = getattr(cp_i, "depth", 0.0) or 0.0
                                    dmap_i = depth_in_contact[fi.name]
                                    if di > dmap_i.get(pi, 0.0):
                                        dmap_i[pi] = di
                                if pj:
                                    parts_in_contact[fj.name].add(pj)
                                    dj = getattr(cp_j, "depth", 0.0) or 0.0
                                    dmap_j = depth_in_contact[fj.name]
                                    if dj > dmap_j.get(pj, 0.0):
                                        dmap_j[pj] = dj
                                break  # next cp_i

            for f in fighters:
                # OOTA check on chassis.
                try:
                    cp = f.node.getPosition()
                except Exception:
                    continue
                if (abs(cp[0]) > half + margin
                        or abs(cp[1]) > half + margin
                        or cp[2] < -0.5):
                    if not f.immobile:
                        f.immobile = True
                        f.disqualify_reason = "oota"
                        sys.stderr.write(
                            f"[damage_director] {f.name} OOTA at "
                            f"t={sim_t:.2f}s\n"
                        )
                    continue

                in_contact = parts_in_contact[f.name]

                for part_id, p in f.parts.items():
                    if p["detached"]:
                        continue
                    try:
                        v = list(p["node"].getVelocity())[:3]
                    except Exception:
                        continue
                    pv = p["prev_v"]
                    dv = math.sqrt((v[0] - pv[0]) ** 2
                                   + (v[1] - pv[1]) ** 2
                                   + (v[2] - pv[2]) ** 2)
                    p["prev_v"] = v
                    # Contact-point gate: only damage parts that are
                    # actually touching the opponent this step.
                    if part_id not in in_contact:
                        continue
                    # Opt-in: score chassis/weapon strikes on real penetration
                    # depth (jitter-free) where a witness exists; wheels
                    # (load-bearing) and any contact without depth (XPBD / old
                    # wire) keep the mass*|Δv| proxy.
                    impulse = p["mass"] * dv
                    if use_depth and not part_id.endswith("wheel"):
                        d_pen = depth_in_contact[f.name].get(part_id, 0.0)
                        if d_pen > 0.0:
                            impulse = p["mass"] * depth_scale * d_pen
                    if part_id == "chassis":
                        thr = th["chassis"]
                    elif part_id.endswith("wheel"):
                        thr = th["wheel"]
                    else:
                        thr = th["weapon"]
                    if impulse > thr:
                        # Damage cooldown: a part can only take one
                        # damage tick per DAMAGE_COOLDOWN_S. A single
                        # contact event spans ~3-5 physics steps, all
                        # at similar impulse; without rate limiting we'd
                        # apply the damage 3-5x and parts would explode
                        # off in the first 30 ms of contact.
                        DAMAGE_COOLDOWN_S = 0.20
                        if (sim_t - p["last_damage_t"]) < DAMAGE_COOLDOWN_S:
                            continue
                        before = p["hp"]
                        p["hp"] -= (impulse - thr)
                        p["last_damage_t"] = sim_t
                        if (impulse - thr) > 0.5 or p["hp"] <= 0:
                            sys.stderr.write(
                                f"[damage_director] {f.name}.{part_id} "
                                f"HIT impulse={impulse:.2f}J "
                                f"(thr={thr:.2f}) "
                                f"hp={before:.1f}->{p['hp']:.1f} "
                                f"t={sim_t:.2f}s\n"
                            )
                        if p["hp"] <= 0 and not p["detached"]:
                            detach_part(f, part_id)

                # Immobilization check: too many wheels lost OR chassis
                # dead.
                lost = sum(1 for k, p in f.parts.items()
                           if k.endswith("wheel") and p["detached"])
                chassis_dead = f.parts.get("chassis", {}).get("hp", 1) <= 0
                if (lost >= fail_wheels or chassis_dead) and not f.immobile:
                    f.immobile = True
                    f.disqualify_reason = ("chassis_dead" if chassis_dead
                                           else f"lost_{lost}_wheels")
                    sys.stderr.write(
                        f"[damage_director] {f.name} IMMOBILIZED at "
                        f"t={sim_t:.2f}s ({f.disqualify_reason})\n"
                    )

            # Match-end decision.
            alive = [f for f in fighters if not f.immobile]
            if len(alive) == 1 and len(fighters) >= 2:
                match_over = True
                winner = alive[0].name
                losers = [f for f in fighters if f.immobile]
                win_reason = losers[0].disqualify_reason if losers else "ko"
                sys.stderr.write(
                    f"[damage_director] MATCH OVER at t={sim_t:.2f}s "
                    f"winner={winner} reason={win_reason}\n"
                )
            elif len(alive) == 0 and len(fighters) >= 2:
                match_over = True
                winner = "draw"
                win_reason = "double_ko"
                sys.stderr.write(
                    f"[damage_director] MATCH OVER at t={sim_t:.2f}s "
                    f"draw (both immobilized)\n"
                )
            elif sim_t >= cfg["timer_s"]:
                match_over = True
                # Tiebreak by most wheels remaining.
                if len(fighters) >= 2:
                    wheels = {f.name: sum(1 for k, p in f.parts.items()
                                           if k.endswith("wheel")
                                           and not p["detached"])
                              for f in fighters}
                    best = max(wheels, key=lambda n: wheels[n])
                    if wheels[best] == min(wheels.values()):
                        winner = "draw"
                        win_reason = "timeout_tie"
                    else:
                        winner = best
                        win_reason = "timeout_wheel_count"
                sys.stderr.write(
                    f"[damage_director] MATCH OVER at t={sim_t:.2f}s "
                    f"reason=timeout winner={winner}\n"
                )

        write_state()

        if match_over and not end_writeout_done:
            out_path = Path(_resolve_scratch_output(cfg["output"]))
            sys.stderr.write(f"[damage_director] writing scorecard -> {out_path}\n")
            try:
                out_path.parent.mkdir(parents=True, exist_ok=True)
                with out_path.open("w", encoding="utf-8") as fh:
                    json.dump({
                        "match_t_s": round(sim_t, 2),
                        "winner": winner,
                        "win_reason": win_reason,
                        "fighters": [
                            {
                                "name": f.name,
                                "immobile": f.immobile,
                                "disqualify_reason": f.disqualify_reason,
                                "broken_parts": list(f.broken),
                                "wheels_remaining": sum(
                                    1 for k, p in f.parts.items()
                                    if k.endswith("wheel")
                                    and not p["detached"]
                                ),
                                "weapon_intact": (
                                    "weapon" not in f.broken
                                    if "weapon" in f.parts else None),
                                "part_hp": {
                                    k: round(p["hp"], 1)
                                    for k, p in f.parts.items()
                                },
                            }
                            for f in fighters
                        ],
                    }, fh, indent=2)
                sys.stderr.write(
                    f"[damage_director] wrote {out_path}\n"
                )
            except Exception as exc:
                sys.stderr.write(
                    f"[damage_director] writeout failed: {exc}\n"
                )

            # Loud, easy-to-spot winner banner. Goes to stderr so it
            # shows up in the OmniSim console even when stdout is
            # buffered.
            wins_line = (f"  WINNER:  {winner.upper()}" if winner
                         else "  WINNER:  (draw)")
            survivors = [f for f in fighters if not f.immobile]
            losers = [f for f in fighters if f.immobile]
            roster_lines = []
            for f in fighters:
                wheels_left = sum(1 for k, p in f.parts.items()
                                  if k.endswith("wheel")
                                  and not p["detached"])
                weapon_status = ("weapon ok" if "weapon" not in f.broken
                                 and "weapon" in f.parts
                                 else ("no weapon" if "weapon" not in f.parts
                                       else "weapon GONE"))
                state = "MOBILE" if not f.immobile else "DOWN"
                roster_lines.append(
                    f"  {f.name:<6} [{state:<6}] "
                    f"wheels={wheels_left}/4  {weapon_status}  "
                    f"chassis HP={f.parts.get('chassis', {}).get('hp', 0):.0f}"
                )
            banner = (
                "\n"
                + "=" * 60 + "\n"
                + "  MATCH OVER\n"
                + wins_line + "\n"
                + f"  reason:  {win_reason}\n"
                + f"  t:       {sim_t:.2f}s\n"
                + "-" * 60 + "\n"
                + "\n".join(roster_lines) + "\n"
                + "=" * 60 + "\n"
            )
            sys.stderr.write(banner)
            sys.stdout.write(banner)
            sys.stdout.flush()
            end_writeout_done = True

    return 0


if __name__ == "__main__":
    sys.exit(main())
