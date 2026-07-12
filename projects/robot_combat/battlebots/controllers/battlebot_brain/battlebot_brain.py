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

"""battlebot_brain — strategy-aware combat AI for BattleBot.proto robots.

This is the BattleBots-era successor to `husky_hunt`. Same state-machine
core (SEEK / CHARGE / REVERSE / PIVOT / VICTORY), plus:

  * Weapon control — spools up the `weapon_motor` device when in range,
    deploys hammer/flipper at impact-likely moments.
  * Pluggable strategies — `aggressor`, `control`, `opportunist`. Each
    biases the state machine differently (range thresholds, charge
    commitment, retreat rules).
  * Opponent autoselect — if no `--opponent` is given, picks the
    nearest robot whose name isn't ours. Lets one controller serve both
    duel and rumble worlds.
  * Match-director awareness — reads `customData` of the DEF DIRECTOR
    supervisor to know when the match is over (so it stops driving
    instead of trying to ram a victorious opponent that already won).

CLI args (all optional, position-independent):
  --opponent <name>     pick a specific target by name field
  --strategy <name>     aggressor|control|opportunist (default aggressor)
  --weapon-spin <0..1>  weapon throttle when engaged (default 1.0)
  --debug               extra log lines per state change

Log: $BATTLEBOT_LOG_DIR/battlebot_<self_name>.log (default _scratch/)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

try:
    from omnisim import Supervisor as _RobotImpl
except Exception:
    from omnisim import Robot as _RobotImpl


WHEEL_NAMES = (
    "front_left_wheel_motor",
    "front_right_wheel_motor",
    "rear_left_wheel_motor",
    "rear_right_wheel_motor",
)
WEAPON_MOTOR_NAME = "weapon_motor"


# ---------------------------------------------------------------------------
# Strategy presets. Each is a small dict of tunings that overlay the defaults.
# ---------------------------------------------------------------------------

DEFAULT_TUNING = {
    "seek_speed":      18.0,
    "charge_speed":    28.0,
    "reverse_speed":   14.0,
    "pivot_speed":     10.0,
    "spin_gain":        6.0,
    "align_tight":      0.20,
    "align_loose":      0.45,
    "charge_range_m":   4.0,
    "disengage_m":      6.0,
    "impact_dv":        1.2,
    "stalemate_range":  1.5,
    "stalemate_speed":  0.30,
    "stalemate_s":      1.0,
    "reverse_s":        1.0,
    "pivot_s_max":      1.2,
    "victory_window_s": 3.0,
    "victory_disp_m":   0.3,
    "victory_speed":    0.15,
    "victory_min_m":    1.0,
    "victory_grace_s":  6.0,
}

STRATEGY_OVERRIDES = {
    "aggressor": {
        # Charges hard, low align tolerance to find any opening.
        "charge_range_m":  5.0,
        "align_tight":     0.30,
        "reverse_s":       0.6,
    },
    "control": {
        # Slower, prefers to push opponents into hazards. Less commitment
        # to head-on charges; bigger reverse window after impact.
        "seek_speed":      14.0,
        "charge_speed":    20.0,
        "charge_range_m":  3.0,
        "reverse_s":       1.6,
        "stalemate_s":     0.6,
    },
    "opportunist": {
        # Waits for opponents to flank each other; charges only when
        # well-aligned. Best in rumble (multiple fighters).
        "charge_range_m":  3.0,
        "align_tight":     0.12,
        "reverse_s":       1.2,
    },
}


def merge_tuning(strategy: str) -> dict:
    tuning = dict(DEFAULT_TUNING)
    tuning.update(STRATEGY_OVERRIDES.get(strategy, {}))
    return tuning


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="battlebot_brain", add_help=False)
    ap.add_argument("--opponent", default=None)
    ap.add_argument("--strategy", default="aggressor",
                    choices=list(STRATEGY_OVERRIDES.keys()))
    ap.add_argument("--weapon-spin", type=float, default=1.0)
    # Weapon mode:
    #   spin  -- continuously drive the weapon motor forward (good for
    #            spinners that need to build up RPM and stay spun up)
    #   pulse -- hold the weapon at rest position until the opponent is
    #            within `pulse-range`, then drive it forward once. Good
    #            for flippers / hammers that fire a single impulse and
    #            need to reset between strikes.
    ap.add_argument("--weapon-mode", default="spin",
                    choices=["spin", "pulse"])
    ap.add_argument("--pulse-range", type=float, default=1.0,
                    help="When weapon-mode=pulse, opponent distance "
                         "(metres) below which the weapon fires.")
    ap.add_argument("--debug", action="store_true")
    # Be permissive: OmniSim sometimes passes extras.
    return ap.parse_known_args(argv)[0]


def _yaw_from_orientation(orient) -> float:
    if orient is None or len(orient) < 9:
        return 0.0
    return math.atan2(orient[3], orient[0])


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _xy_mag(v) -> float:
    return math.hypot(v[0], v[1])


def _scan_robots(supervisor) -> list:
    """Return all named children of the root that aren't us — used to
    pick an opponent when --opponent wasn't passed."""
    out = []
    try:
        root = supervisor.getRoot()
        children = root.getField("children")
        for i in range(children.getCount()):
            try:
                node = children.getMFNode(i)
                nf = node.getField("name")
                if nf is None:
                    continue
                out.append((nf.getSFString(), node))
            except Exception:
                continue
    except Exception:
        pass
    return out


def _find_target(supervisor, self_name: str, opponent_name: str | None):
    candidates = _scan_robots(supervisor)
    if opponent_name:
        for nm, node in candidates:
            if nm == opponent_name:
                return node
        return None
    # Auto-select: closest robot that isn't us, the match director, the
    # broadcast director, the harness, or any environmental Solid.
    skip_prefixes = ("match_director", "broadcast_director",
                     "harness_supervisor", "battlebox", "dropped_box_",
                     "rectangle arena", "wall", "screw_", "pusher_",
                     "killsaw", "floor", "SUN")
    self_node = supervisor.getSelf()
    if self_node is None:
        return None
    try:
        my_pos = self_node.getPosition()
    except Exception:
        return None
    best = None
    best_d = float("inf")
    for nm, node in candidates:
        if nm == self_name:
            continue
        if any(nm.startswith(p) for p in skip_prefixes):
            continue
        try:
            p = node.getPosition()
        except Exception:
            continue
        d = math.hypot(p[0] - my_pos[0], p[1] - my_pos[1])
        if d < best_d:
            best_d = d
            best = node
    return best


def main() -> int:
    args = parse_args(sys.argv[1:])
    tuning = merge_tuning(args.strategy)

    robot = _RobotImpl()
    time_step = int(robot.getBasicTimeStep())
    dt = time_step / 1000.0

    # Derive own name for logging + opponent autoselect.
    self_name = "bot"
    try:
        s = robot.getSelf()
        if s is not None:
            nf = s.getField("name")
            if nf is not None:
                self_name = nf.getSFString() or self_name
    except Exception:
        pass

    log_dir = Path(os.environ.get("BATTLEBOT_LOG_DIR")
                   or os.path.join(os.environ.get("OMNISIM_HOME")
                                   or str(Path(__file__).resolve().parents[5]), "_scratch"))
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        log_f = (log_dir / f"battlebot_{self_name}.log").open(
            "w", buffering=1, encoding="utf-8")
    except OSError:
        log_f = None

    def log(msg: str) -> None:
        line = f"[{self_name}|{args.strategy}] {msg}"
        sys.stderr.write(line + "\n")
        if log_f is not None:
            log_f.write(line + "\n")

    log(f"start; opponent={args.opponent or 'auto'} weapon_spin={args.weapon_spin:.2f}")

    # Bind wheel motors.
    motors = {}
    for name in WHEEL_NAMES:
        try:
            m = robot.getDevice(name)
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            motors[name] = m
        except Exception as exc:
            log(f"wheel motor {name!r} missing: {exc}")

    # Bind weapon motor (optional — wedge has none).
    weapon = None
    try:
        weapon = robot.getDevice(WEAPON_MOTOR_NAME)
        if weapon is not None:
            weapon.setPosition(float("inf"))
            weapon.setVelocity(0.0)
            log("weapon motor bound")
    except Exception:
        weapon = None

    def set_wheels(left: float, right: float) -> None:
        cap = tuning["charge_speed"]
        left = max(-cap, min(cap, left))
        right = max(-cap, min(cap, right))
        try:
            motors["front_left_wheel_motor"].setVelocity(left)
            motors["rear_left_wheel_motor"].setVelocity(left)
            motors["front_right_wheel_motor"].setVelocity(right)
            motors["rear_right_wheel_motor"].setVelocity(right)
        except Exception:
            pass

    def set_weapon(throttle: float) -> None:
        """Throttle -1..1 of weapon's max velocity. Negative values
        drive the weapon backward (useful for resetting a pulse-style
        flipper arm to its rest position)."""
        if weapon is None:
            return
        try:
            max_v = weapon.getMaxVelocity()
        except Exception:
            max_v = 60.0
        try:
            weapon.setVelocity(max_v * max(-1.0, min(1.0, throttle)))
        except Exception:
            pass

    # Hunt for the damage / match director so we can detect match-over
    # states. We accept either name for backward compatibility.
    director_node = None
    DIRECTOR_NAMES = ("damage_director", "match_director")
    def director_state() -> dict | None:
        nonlocal director_node
        if director_node is None:
            for nm, node in _scan_robots(robot):
                if nm in DIRECTOR_NAMES:
                    director_node = node
                    break
        if director_node is None:
            return None
        try:
            raw = director_node.getField("customData").getSFString()
        except Exception:
            return None
        if not raw:
            return None
        try:
            return json.loads(raw)
        except Exception:
            return None

    def own_disabled() -> bool:
        """Director sets a per-bot 'eliminated' flag (KO/OOTA) or
        'broken' map (parts lost) in its broadcast state. We also stop
        driving once the match is declared over."""
        st = director_state()
        if not st:
            return False
        elim = st.get("eliminated") or []
        if self_name in elim:
            return True
        if st.get("match_over") is True:
            return True
        return False

    if not motors:
        log("no wheel motors found; idling")
        while robot.step(time_step) != -1:
            pass
        return 0

    target_node = None
    state = "SEEK"
    state_entered_t = 0.0
    sim_t = 0.0
    start_t = 0.0
    disabled_logged = False

    prev_self_xy_speed = 0.0
    tgt_history: list[tuple[float, float, float]] = []
    stalemate_started_t: float | None = None

    while robot.step(time_step) != -1:
        sim_t += dt

        if own_disabled():
            if not disabled_logged:
                log(f"DISABLED at t={sim_t:.2f}s (director called match)")
                disabled_logged = True
            set_wheels(0.0, 0.0)
            set_weapon(0.0)
            continue

        if target_node is None:
            target_node = _find_target(robot, self_name, args.opponent)
            if target_node is None:
                set_wheels(0.0, 0.0)
                continue
            try:
                tnf = target_node.getField("name").getSFString()
            except Exception:
                tnf = "?"
            log(f"locked onto {tnf} at t={sim_t:.2f}s")
            start_t = sim_t

        try:
            my_self = robot.getSelf()
            my_pos = my_self.getPosition()
            my_orient = my_self.getOrientation()
            my_vel = my_self.getVelocity()
            tgt_pos = target_node.getPosition()
        except Exception:
            set_wheels(0.0, 0.0)
            continue

        tgt_history.append((sim_t, tgt_pos[0], tgt_pos[1]))
        cutoff = sim_t - tuning["victory_window_s"]
        while tgt_history and tgt_history[0][0] < cutoff:
            tgt_history.pop(0)

        dx = tgt_pos[0] - my_pos[0]
        dy = tgt_pos[1] - my_pos[1]
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        yaw = _yaw_from_orientation(my_orient)
        yaw_err = _wrap_pi(bearing - yaw)

        cur_speed = _xy_mag(my_vel)
        dv = abs(cur_speed - prev_self_xy_speed)
        prev_self_xy_speed = cur_speed

        # Victory check (same shape as husky_hunt; director also has its
        # own KO/OOTA termination, so this only fires when both
        # conditions are clear). We require the target to either be
        # OUT-of-arena OR persistently slow AND below the floor — the
        # in-arena stationary check was too eager and false-positived
        # on a target that was just briefly stuck.
        arena_half = 4.0  # half the 8m BattleBox; if changed in world, update
        if (sim_t - start_t) > tuning["victory_grace_s"] and state != "VICTORY":
            if dist > tuning["victory_min_m"] and len(tgt_history) >= 2:
                xs = [p[1] for p in tgt_history]
                ys = [p[2] for p in tgt_history]
                spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                tgt_speed = 0.0
                tgt_z = 0.0
                try:
                    tgt_speed = _xy_mag(target_node.getVelocity())
                    tgt_z = target_node.getPosition()[2]
                except Exception:
                    pass
                window_full = (sim_t - tgt_history[0][0]) >= tuning["victory_window_s"] * 0.9
                # Require OUT-of-arena OR fallen-through-floor; the
                # match_director handles in-arena KO via its own timer.
                target_out = (abs(tgt_pos[0]) > arena_half
                              or abs(tgt_pos[1]) > arena_half
                              or tgt_z < -0.2)
                if (window_full and target_out
                        and spread < tuning["victory_disp_m"]
                        and tgt_speed < tuning["victory_speed"]):
                    log(f"VICTORY at t={sim_t:.2f}s "
                        f"(target OOTA, spread={spread:.2f}m)")
                    state = "VICTORY"
                    state_entered_t = sim_t

        # Stalemate detection.
        in_close = dist < tuning["stalemate_range"]
        is_slow = cur_speed < tuning["stalemate_speed"]
        if in_close and is_slow and state in ("SEEK", "CHARGE"):
            if stalemate_started_t is None:
                stalemate_started_t = sim_t
            elif (sim_t - stalemate_started_t) >= tuning["stalemate_s"]:
                log(f"STALEMATE at t={sim_t:.2f}s -> REVERSE")
                state = "REVERSE"
                state_entered_t = sim_t
                stalemate_started_t = None
        else:
            stalemate_started_t = None

        prev_state = state
        if state == "SEEK":
            if dist < tuning["charge_range_m"] and abs(yaw_err) < tuning["align_tight"]:
                state = "CHARGE"
                state_entered_t = sim_t
        elif state == "CHARGE":
            if dv > tuning["impact_dv"]:
                state = "REVERSE"
                state_entered_t = sim_t
                log(f"IMPACT at t={sim_t:.2f}s (dv={dv:.2f}, dist={dist:.2f})")
            elif dist > tuning["disengage_m"]:
                state = "SEEK"
                state_entered_t = sim_t
        elif state == "REVERSE":
            if (sim_t - state_entered_t) >= tuning["reverse_s"]:
                state = "PIVOT"
                state_entered_t = sim_t
        elif state == "PIVOT":
            if abs(yaw_err) < tuning["align_loose"] or \
                    (sim_t - state_entered_t) >= tuning["pivot_s_max"]:
                state = "SEEK"
                state_entered_t = sim_t

        if state != prev_state and args.debug:
            log(f"state {prev_state}->{state} at t={sim_t:.2f}s "
                f"(dist={dist:.2f}m, yaw_err={math.degrees(yaw_err):.0f}deg)")

        # Drive.
        #
        # Steering policy for forward states (SEEK / CHARGE):
        #   - If yaw_err is large (>= pivot_threshold), do a pure
        #     in-place pivot. The previous "forward - spin / forward +
        #     spin" formula tried to translate and rotate at the same
        #     time; once yaw_err exceeds ~30°, the differential
        #     swamps the forward term and the bot stalls in a slow
        #     curve. A clean pivot is faster.
        #   - Once aligned, drive forward symmetrically. Small residual
        #     yaw_err is corrected by a proportional differential bias.
        #
        # Weapon throttle policy:
        #   - spin mode (default): drive weapon at args.weapon_spin in
        #     SEEK/CHARGE, half-throttle in REVERSE/PIVOT. Suited to
        #     spinners that need to maintain RPM.
        #   - pulse mode: hold weapon at -args.weapon_spin (drive back
        #     to rest position) unless opponent is inside
        #     args.pulse_range AND we're in CHARGE — then fire forward
        #     at +args.weapon_spin. Suited to flippers / hammers.
        pivot_threshold = tuning.get("pivot_threshold", 0.45)

        def _weapon_throttle_for(curr_state: str) -> float:
            if args.weapon_mode == "pulse":
                # Fire only when actually engaging the opponent up close.
                if (curr_state == "CHARGE"
                        and dist < args.pulse_range):
                    return args.weapon_spin
                # Otherwise drive weapon backward (back to rest).
                return -args.weapon_spin
            # spin mode (default).
            if curr_state == "CHARGE":
                return args.weapon_spin
            if curr_state == "SEEK":
                return (args.weapon_spin
                        if dist < tuning["disengage_m"] else 0.3)
            if curr_state in ("REVERSE", "PIVOT"):
                return args.weapon_spin * 0.5
            return 0.0  # VICTORY / unknown

        if state == "VICTORY":
            set_wheels(-tuning["pivot_speed"] * 0.5, tuning["pivot_speed"] * 0.5)
            set_weapon(_weapon_throttle_for("VICTORY"))
        elif state in ("SEEK", "CHARGE"):
            base = tuning["seek_speed"] if state == "SEEK" else tuning["charge_speed"]
            if abs(yaw_err) >= pivot_threshold:
                sign = 1.0 if yaw_err > 0 else -1.0
                set_wheels(-sign * tuning["pivot_speed"],
                           sign * tuning["pivot_speed"])
            else:
                spin = tuning["spin_gain"] * yaw_err
                set_wheels(base - spin, base + spin)
            set_weapon(_weapon_throttle_for(state))
        elif state == "REVERSE":
            set_wheels(-tuning["reverse_speed"], -tuning["reverse_speed"])
            set_weapon(_weapon_throttle_for("REVERSE"))
        elif state == "PIVOT":
            sign = 1.0 if yaw_err > 0 else -1.0
            set_wheels(-sign * tuning["pivot_speed"], sign * tuning["pivot_speed"])
            set_weapon(_weapon_throttle_for("PIVOT"))

    return 0


if __name__ == "__main__":
    sys.exit(main())
