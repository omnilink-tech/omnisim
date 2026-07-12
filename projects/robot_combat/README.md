# Robot Combat

Two combat-robot leagues live here as sibling sub-projects, plus a
shared collection of older head-on/damage-arena test worlds.

```
projects/robot_combat/
├── battlebots/    BattleBots-tribute league — heavyweight bots
│                  faithful to real-world Tombstone, Bite Force,
│                  Hydra, HUGE, ... in a tribute BattleBox arena.
├── orc/           Open Robot Combat — original OmniSim-native
│                  combat-robot sport (stub layout, ready for new
│                  designs).
├── worlds/        Husky-on-Husky brawls + older physics test
│                  worlds (legacy, predates the leagues).
├── controllers/   Shared husky-flavoured controllers
│                  (damage_arena_director, husky_hunt).
└── README.md      This file.
```

Each league is a **self-contained OmniSim project** (worlds + protos +
controllers in one folder) because OmniSim resolves controllers from
the world's project root. Worlds in `battlebots/worlds/` find their
controllers under `battlebots/controllers/`; worlds in `orc/worlds/`
will find theirs under `orc/controllers/`.

## BattleBots league — `battlebots/`

| World | Matchup | Outcome on the canonical run |
|---|---|---|
| [`battlebots/worlds/battlebox_duel.wbt`](battlebots/worlds/battlebox_duel.wbt) | RED vertical-spinner vs BLUE wedge (lightweight reference, ~10 kg) | BLUE wins by chassis_dead at t=122 s |
| [`battlebots/worlds/battlebox_royal_rumble.wbt`](battlebots/worlds/battlebox_royal_rumble.wbt) | All 5 BattleBot weapon archetypes free-for-all | (legacy match_director scorecard) |
| [`battlebots/worlds/battlebox_husky_proving.wbt`](battlebots/worlds/battlebox_husky_proving.wbt) | 2 Huskies in the BattleBox arena (Phase-1 validation) | n/a — arena test |
| [`battlebots/worlds/gravedigger_vs_biteforce.wbt`](battlebots/worlds/gravedigger_vs_biteforce.wbt) | Tombstone tribute (horizontal bar) vs Bite Force tribute (vertical disc) — both heavyweight | mutual weapon destruction, draw at t=240 s |
| [`battlebots/worlds/hydra_vs_gravedigger.wbt`](battlebots/worlds/hydra_vs_gravedigger.wbt) | Hydra tribute (pneumatic flipper, `--weapon-mode pulse`) vs Gravedigger | Hydra breaks Gravedigger's bar at t=3.7 s, draw on wheel count at t=240 s |
| [`battlebots/worlds/huge_vs_bitebot.wbt`](battlebots/worlds/huge_vs_bitebot.wbt) | HUGE tribute (giant wheels + overhead bar) vs BiteBot | draw at timeout, HUGE chassis untouched (wheels did their job) |

### PROTOs

| | |
|---|---|
| [`battlebots/protos/BattleBox.proto`](battlebots/protos/BattleBox.proto) | Configurable combat arena — floor, lexan walls, killsaw, corner pushers, wall screws, optional OOTA pits. |
| [`battlebots/protos/BattleBot.proto`](battlebots/protos/BattleBot.proto) | Parameterised 4WD chassis with a `weaponType` slot: `wedge`, `vertical_spinner`, `horizontal_spinner`, `hammer`, `flipper`. Only used by `battlebox_royal_rumble.wbt` and the legacy worlds; the headliner matchups (Gravedigger, Hydra, HUGE, BiteBot) are inlined directly into the world so the damage director can detach parts via `Supervisor.parent.remove()`. |

### Controllers

| | |
|---|---|
| [`battlebots/controllers/battlebot_brain/`](battlebots/controllers/battlebot_brain) | Strategy-aware combat AI (`aggressor` / `control` / `opportunist`). Supports `--weapon-mode {spin,pulse}` so flippers/hammers hold their arm at rest until contact range. |
| [`battlebots/controllers/battlebot_damage_director/`](battlebots/controllers/battlebot_damage_director) | Physical damage director: tracks per-part HP via `Solid.getContactPoints()`, physically detaches broken wheels/weapons via Supervisor (200 ms cooldown per part to avoid one-contact cascades), declares match over when a bot loses 3 wheels or chassis HP reaches 0. |
| [`battlebots/controllers/match_director/`](battlebots/controllers/match_director) | Legacy judge-scoring referee — 3-min clock, KO/OOTA detection, BattleBots-style judges' scorecard (damage / aggression / control). Still wired into `battlebox_royal_rumble.wbt`. |
| [`battlebots/controllers/broadcast_director/`](battlebots/controllers/broadcast_director) | Camera-switcher — cuts to whoever just took a hit, slow-mo replay over KO/OOTA, parks on the winner. Off by default in the headliner matchups. |

## ORC league — `orc/`

Open Robot Combat. Stub layout — see [`orc/README.md`](orc/README.md)
for the planned direction. Reuses the same `battlebot_brain` and
`battlebot_damage_director` from `battlebots/controllers/` until
something ORC-specific is needed.

## Legacy worlds — `worlds/`

Husky-on-Husky combat worlds and physics test scenarios that predate
the leagues. Split into `demos/` (polished husky brawls) and `tests/`
(older ODE/Newton head-on collision tests, damage-arena box drops).
See `worlds/demos/` and `worlds/tests/` for full listings.

## Running a match

```bash
"$OMNISIM_BIN" --mode=realtime \
    projects/robot_combat/battlebots/worlds/battlebox_duel.wbt
```

For the heavyweight tribute matchups:

```bash
"$OMNISIM_BIN" --mode=realtime \
    projects/robot_combat/battlebots/worlds/gravedigger_vs_biteforce.wbt
"$OMNISIM_BIN" --mode=realtime \
    projects/robot_combat/battlebots/worlds/hydra_vs_gravedigger.wbt
"$OMNISIM_BIN" --mode=realtime \
    projects/robot_combat/battlebots/worlds/huge_vs_bitebot.wbt
```

For the BattleBox judge-scoring tournament bracket:

```bash
N=4 MATCH_TIMEOUT_S=200 bash scripts/battlebox_tournament.sh
```

For the legacy Husky hunt tournament:

```bash
N=5 MATCH_TIMEOUT_S=180 bash scripts/combat_run.sh
```

> **Caveat (2026-07-09) — these bash runners silently use ODE, not Newton.** Their own
> headers say so: launched from bash, omnisim-bin's embedded interpreter cannot find
> `warp`, so `battlebox_tournament.sh` / `combat_run.sh` SILENTLY run on ODE (and the
> tournament script polls a stale pre-`_scratch` scorecard path). They are kept for the
> legacy ODE bracket flow. For a Newton-correct, scorecard-reading match runner use
> [`scripts/dev/combat_match.py`](../../scripts/dev/combat_match.py) as the primary path
> (run from PowerShell — Newton needs `warp`):
>
> ```
> python scripts/dev/combat_match.py <world.wbt> [timer_s] [vel_smooth] [newton|ode]
> ```

## Adding a new combat demo

1. Decide which league it belongs to (`battlebots/` vs `orc/` vs
   `worlds/` for non-league worlds).
2. Drop the `.wbt` under that league's `worlds/`.
3. If it needs a new controller, add it under the league's
   `controllers/`. Reuse shared infrastructure
   (`projects/default/controllers/drive_forward`,
   `harness_supervisor`, `duel_tracker`, `cuda_particle_pool`)
   whenever possible.
4. Reference the new world from
   `projects/samples/demos/controllers/omnilink_launcher/demos.json`
   if it deserves a launcher entry, and from `DEMOS.md` /
   `WORLDS.md` if it's a top-level showcase.

## Adding a new BattleBots-tribute bot

The headliner matchups (Gravedigger, Hydra, HUGE, BiteBot) are
inlined `Robot{}` nodes — NOT PROTO instances — because OmniSim
refuses to remove nodes from a PROTO body. To add a new bot:

1. Open one of the existing matchup worlds as a template (e.g.
   `gravedigger_vs_biteforce.wbt`).
2. Copy the relevant `DEF MYBOT Robot { ... }` block, give every
   part a world-scope DEF following `<BOTNAME>_FL_WHEEL` /
   `<BOTNAME>_WEAPON` / etc.
3. Tune masses, weapon RPMs/torques, and damage thresholds for the
   new bot in the world's `DEF DIRECTOR ... customData`.
4. Hook the brain with `--weapon-mode pulse` for one-shot weapons
   (flippers, hammers) or leave it on the default `spin` for
   continuous spinners.
