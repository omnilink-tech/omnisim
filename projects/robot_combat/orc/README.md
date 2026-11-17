# ORC — Open Robot Combat

A combat-robot sport developed natively in OmniSim — **not** a tribute
to an existing real-world league. Where the `battlebots/` league uses
asymmetric heavyweight designs faithful to actual real-world bots
(Tombstone, Bite Force, Hydra, HUGE, ...), ORC is the open-format
counterpart: same physical damage system, but the bots, weapons, and
arena are designed for OmniSim from scratch.

## Status

ORC includes open-field, forest and queen-defense experiments, plus the
**Foundry playable encounter**. This is a small game prototype and visual
benchmark inside OmniSim. It does not yet demonstrate an open world,
AAA production quality, or sustained 1080p/60 FPS.

## Foundry local garage and matches

Open **[Launch Foundry.bat](Launch%20Foundry.bat)** on Windows, or run:

```console
python projects/robot_combat/orc/launch_foundry.py
```

The launcher opens a local browser garage. Configure **both robots on this
laptop**, choose a 30-second, one-minute or three-minute match, then press
**Start match**. OmniSim opens the arena and both robots fight autonomously.
The browser shows a periodically refreshed arena image, lost parts, the result,
and the last twelve matches. Use **Rematch** or load a previous matchup's builds
to adjust its strategy. The native arena window provides the moving 3D view.

Each robot has a name, six paint options, a horizontal spinner or vertical drum,
three frame packages, three drive reductions, a 6/8/10 kg rotor, and a fighting
style: **Pressure**, **Flanker**, or **Counter-striker**. Both sides use the same
catalog and can choose identical hardware. Frame and rotor masses change inertia;
frame packages change assumed mount strength; gearing trades torque for speed.
Garage performance figures are unloaded design estimates, not measured speeds.
See [customization assumptions](REALISM.md#garage-builds-and-strategies).

**Save builds** keeps the pair locally. **Export** and **Import** exchange validated
JSON blueprints; they cannot supply controller code or executable paths. Drafts,
the last twelve match summaries, and per-match logs/worlds are saved under
`_scratch/foundry_garage/`. The server listens only on the laptop's loopback
interface. Remote players, accounts and online matchmaking are not implemented.

The 32 m salvage yard has physical perimeter cover,
asphalt, concrete, painted metal, beveled armor, wheel hubs, tread blocks and
industrial signage. The center remains clear for combat and free driving.

### Direct encounter controls

The original `orc_foundry.omniworld` remains available for manual play. In that
world, click the 3D view and press **Enter** to drive ANVIL against RAZOR. The
garage's autonomous matches accept only **C** (camera) and **H** (HUD); rematches
start from the garage.

| Control | Action |
|---|---|
| Enter | Start the encounter after a three-second countdown |
| WASD or arrow keys | Drive and steer ANVIL |
| Space | Toggle ANVIL's weapon |
| Tab | Switch between manual driving and AI demonstration |
| C | Cycle chase, tactical and low overview cameras |
| H | Toggle the HUD |
| R | Reload the world for a fresh encounter |

The shipped world perspective locks free-fly camera movement and mouse
physics manipulation so WASD controls the robot. The game's C key still
changes camera. Input is also relayed through either selected fighter,
so clicking a robot does not cut off the controls.

The encounter ends on measured physical immobilization, leaving the yard,
or the three-minute time limit. A count-out requires ten seconds of attempted
driving without significant translation or rotation, clear of opponent contact.
The AI tries forward, reverse and turning recovery commands while its opponent
makes space. Losing three wheels or exhausting a damage indicator cannot switch
off surviving motors. Motors stop after the result; use C to inspect the
aftermath and R to restart. There is no scripted explosion or automatic
HP drain. Damage uses paired physical contacts and estimated plastic work
above component yield loads, bounded by available impact energy. Motors
use a published electrical model, finite current and power, shared battery
sag, and reaction torque. See [physics assumptions and verification](REALISM.md).
The material failure parameters have **not been validated against real impacts**.
The chassis indicator describes estimated load capacity; hull deformation and
chassis fracture are not implemented and that indicator does not decide a knockout.

Broken wheels and weapons retain their original geometry, collider, mass,
world pose, and all six velocity components. Their owning joint is removed;
one physics rebuild is requested per batch of detachments. The new body's
velocity is staged before finalization, and the engine preserves surviving
joint velocities through the rebuild. This addresses the earlier ORC
pattern where a visually detached part could remain frozen outside the
physics model. The older match directors are not changed by Foundry.

The guarded launcher checks GPU temperature every 0.5 seconds, starts at
or below 65°C, and stops its workload at a sampled **75°C** to leave margin
below an 80°C limit. Loss of the temperature sensor also stops the workload.
On Windows, a job object owns the engine and all child controllers, so
closing a failed wrapper cannot leave them running. This is a sampled
software guard, not a hardware temperature guarantee. **CPU temperature
is not monitored.** The world caps preview rendering at 30 FPS; that cap
is not a measured performance result. Directly loading the world from the
general OmniSim demo browser does **not** activate the thermal guard.

```console
# Watch both robots fight; same GPU protection.
python projects/robot_combat/orc/launch_foundry.py --demo

# Short, real-time physics observation and a 30-second encounter limit.
python projects/robot_combat/orc/launch_foundry.py --demo --headless --seconds 45 --match-seconds 30

# Regenerate/check the authored world, without launching the engine.
python projects/robot_combat/orc/build_foundry.py
python projects/robot_combat/orc/build_foundry.py --check

# Require a full match ending in physical immobilization, with numerical evidence.
python projects/robot_combat/orc/verify_foundry.py --cases match

# Same check with a recorded 1280×720 match at simulation speed and an aftermath image.
python projects/robot_combat/orc/verify_foundry.py --cases match --render
```

Each direct guarded launch writes `engine.log`, `encounter.jsonl` and `run.log`
under a new `_scratch/foundry/<timestamp>/` directory. The encounter log
records real impacts, detachments, rebuild requests, motion samples and
the result. The run log records the guard's observed GPU peak. Controller
configuration uses `ORC_FOUNDRY_DEMO`, `ORC_FOUNDRY_DURATION` and
`ORC_FOUNDRY_OUTPUT`, all set by the launcher.

Full-match verification also saves a machine fingerprint, source hashes and
the final world. It independently checks powered motion, opponent clearance,
the winner's mobility, conserved mass, the plastic-work budget and chassis
clearance above the floor. A time-limit result fails this verification.

Source: [`build_foundry.py`](build_foundry.py),
[`foundry_logic.py`](shared/foundry_logic.py), and the
`orc_foundry_pilot` / `orc_foundry_director` controllers. The generated
world is committed for direct use; change the generator and regenerate
rather than hand-editing its output.

### Validation and remaining work

Validated on 2026-09-13, machine `9722d23d12a3`, RTX 3060 Laptop GPU,
Newton/MuJoCo CPU solver. The engine was rebuilt from the working tree;
[the evidence record](verification/full_match_cpu.json) identifies its binary
and source hashes rather than attributing these changes to an older commit.

- A full match ended at simulation time **144.728 s** with ANVIL mechanically
  immobilized after overturning and losing a wheel and its weapon. During its
  final ten seconds of powered recovery, its measured translation span was
  **8.15 cm** and rotation span **4.05°**. RAZOR remained mobile. This is a
  physical count-out with small reaction motion, not absolute zero movement.
- The match passed independent motion, contact, mass, energy-budget and floor
  checks; the engine reported zero errors and zero warnings. GPU peak: **64°C**.
- **33 Foundry unit tests** and the broad engine-free suite (**1,329 passed,
  11 skipped**) passed. Two new native-engine regressions verify mixed mesh
  colliders through a rebuild and friction-dependent sliding.
- The five physical probes passed: weapon spin, grounded drum spin, driving
  and braking, rebuild conservation, and detached-weapon conservation.

These are individual verified runs, not a claim that every AI match ends in
a knockout or that real-world breakage has been validated. See [REALISM.md](REALISM.md)
for the measurements, adjudication tolerances and remaining physics limits.
Two filmed follow-up matches reached the time limit and correctly failed the
knockout check; their complete recordings are retained in the evidence record.
The highest sampled GPU temperature across these runs was **67°C**.

Native screenshots omit the supervisor HUD overlay; the garage supplies its
own match status beside its preview. Keyboard routing is
covered by engine-free tests; no human input playtest or sustained frame-rate
benchmark is claimed. The next production work is collision-aware camera
handling, richer combat AI, audio and impact effects, better environment
assets, and measured frame-time tuning. World streaming, progression,
mid-match saves, multiplayer and an open-world population are outside this prototype.

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

1. Author the bot inline in a new `worlds/<matchup>.omniworld` using direct
   `Robot{}` nodes (not a PROTO instance) — PROTO encapsulation
   prevents the damage director from detaching parts. See
   `battlebots/worlds/battlebox_duel.omniworld` for the reference pattern.
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

`worlds/orc_queen_defense.omniworld` runs an alternative ORC ruleset: each
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

To author a new queen-defense match, copy `orc_queen_defense.omniworld` and
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
