# ORC and the OmniSim robot combat pipeline

Review date: 13 September 2026. Checkout: OmniSim 8.5.1, `main` at `3cc84d6c8`.

**Recommendation: build a compact, playable ORC sandbox on OmniSim. Treat a persistent, seamless open world as a subsequent engineering phase.** The existing work provides a useful simulation and content foundation, but the combat systems need consolidation and physical correctness fixes before they can support a realistic game reliably.

This assessment assumes player-controlled wheeled combat robots, autonomous opponents, explorable terrain, and mechanical weapons. A game with ranged weapons, airborne combat, or independently balanced fighting humanoids would require additional combat systems. The presence of those robot types elsewhere in the simulator does not establish their readiness for combat.

## What was inspected

The review traced ORC, the BattleBots projects, the older Husky combat worlds, their controllers and directors, the general harness damage tracker, match/stability/realism tools, procedural world generation, rendering documentation, and the Newton scene mutation and contact implementations. It includes bounded engine observations, a short completed ORC match, a short completed BattleBox match, and a controlled detachment experiment with a rebuild comparison. It does not certify every weapon/world combination, rendering frame rate, multiplayer, or long-session stability.

## What exists today

| Stage | Existing implementation | Implication for a game |
|---|---|---|
| ORC scenarios | A 20 m open field with four fighters, a 40 m forest battlefield with six fighters, and queen defense with two queens plus four fighters | Useful authored scenarios; these are bounded maps with match clocks and out-of-bounds rules |
| Robot construction | Inline articulated robots, wheel motors, collider geometry, physical masses, a parameterized BattleBot PROTO | Strong starting point for a modular robot roster; inline duplication and world-scope part names need a shared authoring system |
| Mechanical weapons | Wedges, vertical and horizontal spinners, hammers, flippers, continuous and pulsed motor control | Enough variety for a first combat game; individual weapons need impact and energy-transfer validation |
| Combat AI | Pursue, charge, reverse, pivot, victory; three tuning presets | A starting behavior controller, with substantial work needed for terrain navigation and tactics |
| ORC damage | Per-part HP, contact-gated deductions, broken-wheel/weapon records, immobilization, match output | A functioning gameplay damage loop, with a confirmed gap between scene detachment and physical detachment |
| Broader damage system | Dents, appearance changes, procedural fragments, repair, wheel behavior gates, damage events | Valuable reusable code in the general harness tracker; it is not all wired into ORC |
| Environment generation | Seeded forest/desert/urban/warehouse/apartment/Mars recipes, terrain and scatter primitives | Supports varied maps; generation is not a running world-streaming system |
| Presentation and automation | wgpu materials/lighting, camera controllers, capture/cinema tools, harness/MCP control | Useful development and presentation infrastructure; player camera, HUD, input flow and impact audio still need game-specific work |
| Verification | Load/stability checks, scorecards, optional contact traces, damage regression scripts | Good instruments exist, but several current success criteria do not certify combat correctness |

Sources: [ORC worlds](O:/omnisim/projects/robot_combat/orc/worlds), [BattleBot](O:/omnisim/projects/robot_combat/battlebots/protos/BattleBot.proto:30), [general damage tracker](O:/omnisim/projects/default/controllers/harness_supervisor/damage_tracker.py:700), [world generation](O:/omnisim/docs/developer/omniworld-user-guide.md:7), [renderer status](O:/omnisim/docs/developer/wgpu-renderer-status.md:1).

## Findings that should govern the build order

**1. Physical detachment is currently incomplete. This was reproduced, not inferred only from source.**

The director exports a part, imports a free Solid, assigns a velocity, and removes its original joint. It does not rebuild the Newton model. In the controlled ORC experiment, a weapon was detached at approximately t=2 s and the original scene node disappeared. The new part then remained at exactly the same position through t=5.2 s, even though it reported a linear velocity of approximately `(1.635, -0.747, 0.077)` m/s. The match output nevertheless marked the weapon broken.

In a separate diagnostic run, a temporary observer requested `simulationRebuildPhysics()` at t=3 s. The new part subsequently moved, reaching approximately 3.05 m displacement by the end of the observation. This verifies a usable corrective mechanism for this case. It does not establish hitch-free destruction, conservation of momentum, or safety across every articulation.

The same director also explicitly zeroes the detached part's angular velocity. Even with registration repaired, a spinning weapon would lose its inherited spin. Both ORC and the BattleBots director share this behavior.

The engine's existing deletion probe documents the corresponding old-collider issue: removal from the scene alone can leave collision geometry in the compiled physics model. The current rebuild implementation also refuses worlds containing Cloth/SoftBody particles or a GranularBed. Combining those systems with frequent combat breakage will need a more complete mutation design.

Sources: [detachment code](O:/omnisim/projects/robot_combat/orc/controllers/battlebot_damage_director/battlebot_damage_director.py:509), [comparison measurements](O:/omnisim/_scratch/orc_audit_20260913/detachment_comparison.json), [existing deletion probe](O:/omnisim/tests/benchmarks/omnibench/lane4/capabilities.py:4191), [rebuild implementation](O:/omnisim/src/omnisim/engine/OmSimulationWorld.cpp:225), [rebuild restrictions](O:/omnisim/src/omnisim/engine/OmSimulationWorld.cpp:382).

**2. The bots do not yet behave like open-world combatants.**

The ORC brain chooses its target once, usually from the world's explicit `--opponent`, and steers toward the target's exact supervisor-reported position. It has no integrated path planner, line-of-sight perception, terrain assessment, or general team-aware retargeting. The three strategies mostly change numerical thresholds. Cover can block movement without producing a plan to go around it.

The brain also retains `arena_half = 4.0`, inherited from the 8 m BattleBox. ORC's fields are 20 m and 40 m wide. Its local victory heuristic can therefore treat a stationary target in a valid ORC location as outside the arena. The open-field and forest directors use last-bot-standing rules despite the blue/red team presentation; queen defense contains a separate team-aware win condition. The queen's behavior is a random walker with a boundary safeguard, and protectors target the enemy queen rather than performing a defensive role.

Navigation and avoidance code elsewhere in OmniSim is reusable, but it is not a combat navigation system already connected to these brains.

Sources: [target selection](O:/omnisim/projects/robot_combat/orc/controllers/battlebot_brain/battlebot_brain.py:183), [pursuit and boundary logic](O:/omnisim/projects/robot_combat/orc/controllers/battlebot_brain/battlebot_brain.py:351), [queen controller](O:/omnisim/projects/robot_combat/orc/controllers/queen_walker/queen_walker.py:15), [existing mobile avoidance](O:/omnisim/projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py:5302).

**3. The damage system is a gameplay approximation, not a calibrated material failure model.**

Damage uses body mass multiplied by the change in linear velocity, subtracts a threshold, and applies a cooldown. An optional variant uses penetration depth. The contact gate matches points from two fighters within a 10 cm tolerance, with nearest-part fallbacks. It only attributes fighter-to-fighter damage in this director; it does not provide a general damage pipeline for walls, falls, or arena hazards.

This can support convincing gameplay after calibration. It does not presently establish armor penetration, material yield, structural stress, reliable impact-energy accounting, or a correspondence between visual dents and changing collision shapes. In particular, `mass * delta_velocity` is momentum/impulse with units N·s, although several labels call it joules. A spinning weapon's local contact speed also depends on angular motion, which the linear-velocity proxy does not directly model.

The CPU contact runtime already reads actual body pairs, contact locations, normals, penetration, and force magnitude. Connecting this richer signal to a unified damage service is a better next step than adding more arbitrary HP values. Integrating contact forces over time and tracking rotational energy would still need validation; it would not automatically become a material fracture model.

Sources: [contact matching and damage](O:/omnisim/projects/robot_combat/orc/controllers/battlebot_damage_director/battlebot_damage_director.py:750), [native contact data](O:/omnisim/src/omnisim/physics/omnisim_newton_runtime.py:2890), [visual geometry replacement](O:/omnisim/projects/default/controllers/harness_supervisor/damage_tracker.py:1743).

**4. The pipeline has diverged into several partially connected systems.**

ORC and BattleBots have separate copies of their brain and damage director. BattleBots has the per-bot tuning and realism trace support that ORC lacks; ORC has queen rules that BattleBots lacks. The richer general damage tracker is a third implementation. A new feature can therefore work in one lane and be absent in another.

The legacy `match_director` scores impacts and falls back to impact counts when its hardcoded damage API is unavailable. Its events also differ from the physical damage director's broadcast state. The broadcast controller's named replay is a camera cut over the eliminated robot, not a buffered replay of the preceding collision. Arena screws and the killsaw are static colliders in the arena PROTO rather than powered hazards. These are usable pieces, but their names and README descriptions overstate some current behavior.

The README still calls ORC a stub and contains obsolete `.wbt` authoring and ODE runner advice. Those descriptions should be reconciled with the live code before becoming the game's developer contract.

Sources: [ORC README](O:/omnisim/projects/robot_combat/orc/README.md:10), [legacy scoring](O:/omnisim/projects/robot_combat/battlebots/controllers/match_director/match_director.py:149), [broadcast behavior](O:/omnisim/projects/robot_combat/battlebots/controllers/broadcast_director/broadcast_director.py:205), [arena hazards](O:/omnisim/projects/robot_combat/battlebots/protos/BattleBox.proto:150).

**5. The open-world game layer remains substantial new work.**

I found no complete ORC implementation of persistent world entities, sector streaming, distant-agent simulation, save-game progression, robot inventory/loadouts, salvage economics, player onboarding, or multiplayer authority/prediction. Simulator state snapshots and HTTP control are useful primitives, but they do not constitute those game systems.

A larger static map is a reasonable intermediate step. A seamless world with continuous spawning and destruction needs stable entity identity, state serialization, bounded active physics, and safe insertion/removal. Running multiple OmniSim processes proves separate worlds can coexist; it does not establish one distributed shared battlefield.

Recommendation: keep detailed physics around the player and relevant encounters, and represent distant activity with cheaper game state. First prove one region with a small roster; later add sector transitions and persistence. The scale limit depends strongly on active articulations, contacts, debris, controller work, and rendering, rather than map area alone.

Sources: [scene spawning contract](O:/omnisim/packages/omnisim-mcp/README.md:171), [reload boundary](O:/omnisim/docs/developer/world-reload-cache.md:24), [procedural generation scope](O:/omnisim/docs/developer/omniworld-user-guide.md:7).

**6. Realism settings and verification need attention before expanding scope.**

The tested ORC worlds declare inherited friction and bounce fields that Newton does not consume. The open-field load finalized and stepped, but failed the requested strict warning gate. The engine reported effective friction 1.0 instead of the authored 0.9, and ignored bounce. This is a content/integration problem, not proof that realistic impacts are impossible. MuJoCo's upstream documentation describes restitution through stiffness/damping settings; OmniSim's existing `bounce` field is not connected to that behavior. [MuJoCo restitution documentation](https://mujoco.readthedocs.io/en/latest/modeling.html#restitution).

The current match runner counts any broken part or immobilized fighter as evidence that ramming dealt damage. Immobilization can also follow an out-of-bounds event, and the runner does not require a Newton sidecar or verify the detached body's motion. Its advertised velocity-smoothing setting is read by the general tracker, not either combat director. The realism reporter is useful, but ORC lacks its trace producer.

The standard runaway watchdog tracks startup robot roots. It passed the frozen-debris test because it did not track the new part. It also could not identify the static floor inside the test worlds' PROTOs. Thus these PASS results have a narrower meaning than combat correctness.

Sources: [strict load log](O:/omnisim/_scratch/orc_audit_20260913/open_field_load_retry.log), [match success condition](O:/omnisim/scripts/dev/combat_match.py:131), [realism reporter](O:/omnisim/scripts/dev/combat_realism_report.py:15), [observer results](O:/omnisim/_scratch/orc_audit_20260913/detachment_observations.jsonl).

## Validation performed and its limits

All new physical observations below were on machine `9722d23d12a3`, Windows 11, AMD Family 25 Model 80 / 16 logical cores, RTX 3060 Laptop GPU; engine SHA-256 prefix `a484486d5ae58db1`, controller library prefix `0bd8db2b5018d7fe`. Physics ran on CPU MuJoCo 3.11.0 through bundled Newton 1.5.0 / Warp 1.16.0. Sidecars confirmed finalized, non-degraded Newton runs. The controller library is older than the engine but passes the IPC nonce compatibility check.

The initial local launch could not use the protected user Warp cache; the audit redirected the cache into its own scratch directory. Python was supplied through the available runtime for these child processes. No machine-wide settings or original combat sources were changed.

| Observation | Result | What it establishes |
|---|---|---|
| ORC open field, short run | 8.31 simulated seconds; robot-root stability check passed | Loads and moves; not a complete match |
| ORC forest war | 4.03 simulated seconds; six robot roots tracked; stability check passed | Short startup/motion evidence only |
| ORC queen defense | 6.40 simulated seconds; five logged HP deduction events; stability check passed | Damage events occur; queen win conditions not certified by this run |
| BattleBox duel, shortened clock | Draw at 15.01 simulated seconds, no HP loss; trace contained no bot-on-bot contacts | Match clock, output and tracing work; this run does not prove weapon effectiveness |
| ORC open field, shortened clock | Draw at 15.00 simulated seconds, small chassis/wheel HP losses, no broken parts | Completed combat bookkeeping with some engagement |
| Forced ORC weapon detachment | Scorecard says broken; new part frozen for 3.2 simulated seconds | Reproduced physical registration defect |
| Forced detachment plus diagnostic rebuild | Part moves after rebuild | A corrective mechanism works in the isolated case |

The first three observations each used a 22-second wall-clock window after the first step, with realtime pacing, no rendering, and a watchdog. They advanced substantially less simulation time than wall time. Instrumentation and Windows scheduling affect these figures, so they are not optimized throughput benchmarks. They also do not support promising a rendered real-time open world. Rendered frame pacing, input latency and a debris-heavy soak remain required.

Raw results: [verified runs](O:/omnisim/_scratch/orc_audit_20260913/verified_results.json), [rebuild run](O:/omnisim/_scratch/orc_audit_20260913/rebuild_results.json), [detachment comparison](O:/omnisim/_scratch/orc_audit_20260913/detachment_comparison.json), [BattleBox realism report](O:/omnisim/_scratch/orc_audit_20260913/battlebox_realism.json).

## Proposed first playable version

Build one explorable forest/industrial region, initially with one player robot and a small active roster. A 100 m region and roughly four to six active robots are proposed scope targets, not measured capacity promises. Use three complementary mechanical weapon types, a repair/salvage location, and patrol, escort or queen-defense encounters. Give the player a responsive chase camera, controls, clear component status, and contact-driven sound/visual feedback.

The play loop should be: leave the workshop, find or accept an encounter, fight using terrain and positioning, suffer meaningful component damage, recover salvage, return and repair or change the robot. That exercises the distinct value of OmniSim and exposes the systems a persistent world will need.

Build in this order:

1. **Make combat dependable.** Unify event and damage contracts; repair physical detachment and inherited spin; migrate world settings; add direct contact attribution and tests for active, removed and detached colliders.
2. **Make it playable.** Add player control/camera/HUD/audio; obstacle-aware navigation, target switching, teams and objective behavior. Preserve the existing simple controller as a baseline opponent.
3. **Make consequences persist.** Shared robot definitions, loadouts, component state, repairs, salvage and save/load. Test that damage and debris survive region transitions correctly.
4. **Prove the performance budget.** Measure rendered play on the target machine, worst-frame timing, input latency, and repeated destruction. Reduce controller polling and bound debris/active bodies before expanding the map.
5. **Expand the world.** Stream sectors and approximate distant activity only after one region works. Add multiplayer as a separate authority, replication and latency project if that is part of the intended product.

Before calling the prototype realistic, require repeatable evidence that impacts reduce the correct component's health, a detached part remains dynamic and inherits linear/angular state, the former attachment stops colliding, obstacles provoke navigation rather than indefinite pushing, valid map positions never trigger false victory, and the complete rendered encounter runs at its chosen frame and simulation budgets.

**Decision:** OmniSim is a credible foundation for an original ORC game. The highest-value next investment is a complete small gameplay loop with trustworthy combat. A large map alone would leave the current correctness and AI gaps exposed.
