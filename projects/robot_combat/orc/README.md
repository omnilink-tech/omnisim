# ORC — Open Robot Combat

A combat-robot sport developed natively in OmniSim — **not** a tribute
to an existing real-world league. Where the `battlebots/` league uses
asymmetric heavyweight designs faithful to actual real-world bots
(Tombstone, Bite Force, Hydra, HUGE, ...), ORC is the open-format
counterpart: same physical damage system, but the bots, weapons, and
arena are designed for OmniSim from scratch.

## Status

Stub layout. The directory is set up the same way as
`projects/robot_combat/battlebots/` (a self-contained OmniSim project)
so the same controllers / damage director patterns can be reused or
forked.

```
projects/robot_combat/orc/
├── worlds/        ORC match worlds
├── protos/        ORC arena + bot PROTOs
├── controllers/   ORC-specific controllers
└── README.md      This file
```

## How ORC differs from BattleBots

| | BattleBots | ORC |
|---|---|---|
| Source | Tribute to real-world BattleBots designs | Original designs developed in OmniSim |
| Weight classes | Fixed at heavyweight (~110 kg) | TBD — likely multiple classes |
| Arena | BattleBox tribute (8 m square, lexan walls, killsaw, corner pushers) | TBD |
| Naming | Tribute names (Gravedigger, BiteBot, ...) | Native names |
| Brain | Shared `battlebot_brain` (works for any 4WD chassis with weapon_motor) | Will reuse `battlebot_brain` until something ORC-specific is needed |
| Damage system | `battlebot_damage_director` from `battlebots/controllers/` | Same — physical damage + part detachment |

## Designing an ORC match

1. Author the bot inline in a new `worlds/<matchup>.wbt` using direct
   `Robot{}` nodes (not a PROTO instance) — PROTO encapsulation
   prevents the damage director from detaching parts. See
   `battlebots/worlds/battlebox_duel.wbt` for the reference pattern.
2. Give each part a world-scope DEF following
   `<BOTNAME>_FL_WHEEL`, `<BOTNAME>_FR_WHEEL`,
   `<BOTNAME>_RL_WHEEL`, `<BOTNAME>_RR_WHEEL`,
   `<BOTNAME>_WEAPON`.
3. Wire each bot's controller to `battlebot_brain` (in
   `battlebots/controllers/`) with the right `--opponent`, `--strategy`,
   and `--weapon-mode` flags.
4. Add a `DEF DIRECTOR Robot { controller "battlebot_damage_director" }`
   with `customData` listing the fighters and tuned HP / impulse
   thresholds for the new mass class.
5. (Optional) Add `DEF TRACKER Robot { controller "duel_tracker" }` to
   write a per-step CSV trace.

## Queen defense mode

`worlds/orc_queen_defense.wbt` runs an alternative ORC ruleset: each
team fields a **queen** plus protectors, and the win condition flips
from "last bot standing" to "team whose queen survives wins".

  * The queen is a non-combatant — same chassis on both sides, only
    the body colour differs (deep blue vs deep red) — driven by the
    `queen_walker` controller, which is the Husky random walker with a
    soft arena fence so she doesn't drive OOTA by accident.
  * Protectors are ordinary `battlebot_brain` fighters with their
    `--opponent` flag pointed at the enemy queen, so the natural
    behaviour is "drive past their protectors and ram their queen."
  * The damage director runs in **queens mode**: customData declares
    `teams` and `queens` maps, and the match ends the instant a queen
    is immobilized (chassis HP gone, ≥3 wheels lost, or OOTA). The
    other team wins regardless of how many of their own bots are still
    up. Both queens immobilized in the same step → draw.

To author a new queen-defense match, copy `orc_queen_defense.wbt` and
swap the protectors / cover layout. The director's customData fields
that switch on queens mode are:

```jsonc
{
  "teams":  {"blue": ["blue_queen", "blue_alpha", "blue_bravo"],
             "red":  ["red_queen",  "red_alpha",  "red_bravo"]},
  "queens": {"blue": "blue_queen", "red": "red_queen"}
}
```

Omit those keys and the director falls back to the standard
last-bot-standing rule.
