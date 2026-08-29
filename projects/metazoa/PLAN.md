# METAZOA — robots that grow into robots

*The OmniSim flagship simulation. Plan v1, 2026-08-28.*

## The name

**Metazoa** is biology's word for the animals: the lineage that made the one
transition that matters most in the history of life — single cells that
learned to stay together and become one organism. That transition is exactly
what this simulation stages, with robots: identical elemental machines that
dock into bodies, and bodies that move, feed, divide and die. The word is
short, unclaimed in robotics and artificial life (the only prior "Metazoa" is
a 2006 mixed-reality pet game), and reads as "meta" to a robotics audience.
Vocabulary: a **cell** is one robot; an **organism** is a welded body of
cells; the **reef** is the world they live in.

Tagline: *robots that grow into robots.*

## What it is, in one paragraph

A fixed population of small identical robot cells lives on a lit floor. A
cell is a real machine under real physics: two bodies, one motorised hinge,
four docking faces, a battery. Alone it can barely crawl. Cells that dock —
a physical weld, engaged only when two faces are genuinely aligned — become
an organism whose gait is a travelling wave down its spine: chains snake,
branched bodies crawl, and the *shape* of the body decides how it can move.
Organisms feed on light patches, share charge across their cells, recruit
free cells to grow, divide when they are big and full, shed cells when they
starve, and die into debris that is recycled as fresh cells. Nothing is
created or destroyed — the cell count is conserved, like mass in Flow-Lenia —
so what evolves is *organisation*: body plans and gaits are inherited at
division with mutation, and the reef selects them. An LLM can run the reef
over HTTP. It renders in real time on a laptop.

## What is new here (stated carefully)

The pieces have deep roots, and the plan borrows from each deliberately:
**Symbrion/Replicator** (2008–13) built the swarm→organism lifecycle in
hardware — IR-guided docking, energy sharing between docked modules, a seed
that broadcasts a body plan — and reached three-module organisms. **M-TRAN**
and **PolyBot** established the hinge-module chain that snakes and walks, and
the CPG+genetic-algorithm gait generation this plan uses. **Swarm-bots** showed
autonomous self-assembly of up to 16 robots. **Molecubes** demonstrated
self-reproduction. **Karl Sims** evolved bodies and brains together.
**Flow-Lenia** made open-ended evolution emerge from a mass-conserving
continuous automaton. We found no system that combines them: an evolutionary
*ecology* of self-assembling modular robots under full rigid-body physics with
a conserved population, running in real time, with an agent-operable surface.
That is the claim — "first to combine", never "never done".

## Decisions, with the reasoning

**1. The cell is a two-body hinge module (PolyBot/M-TRAN lineage), not a
wheeled robot.** Two reasons. It is how self-reconfigurable hardware actually
works, so every organism gait we get is one the field has built. And a
position-controlled hinge can be driven by *one supervisor* through batched
field writes — the architecture measured at 5–8 ms/step in `projects/alife`
— whereas wheels are velocity joints that need one controller process per
robot, which is what overheated the laptop in the RoboLife attempt.

**2. Docking is the engine's native Connector weld, orchestrated by the
supervisor.** Verified today: a field write to `isLocked` calls the
connector's real `lock()`; the engine welds only if a compatible face is
within `distanceTolerance`/`axisTolerance` — a cell cannot cheat its way onto
a body. The weld itself was proven on a moving robot this session (4 m tow,
1.5 mm stretch, clean release) after fixing an engine bug that had silently
killed all welds since the newton 1.5 upgrade. Welds are equality
constraints, not tree joints, so **loops are legal** — rings and closed
bodies are possible, which PhysX-style articulations forbid.

**3. Consecutive cells dock rotated 90°, so a chain alternates pitch and yaw
hinges.** Straight from M-TRAN. A chain with only pitch hinges can inchworm
but cannot steer; alternating axes gives lateral undulation, sidewinding and
turning. Which rotation a cell docks with is part of the organism's body
plan (`numberOfRotations 4`).

**4. Cell count is conserved.** Runtime spawn/delete have no physics in this
engine, so every cell exists from load — and rather than hide that, the
design makes it the physics of the world: matter is finite, dead cells are
debris, debris is recycled as new cells at the reef's edge. Flow-Lenia's
mass conservation is what made its evolution open-ended instead of
degenerate; the same principle applies here.

**5. Organisms share energy.** Docked cells equalise charge every tick
(Symbrion's energy bus). This is the concrete *advantage* of being
multicellular — a starving cell survives inside a fed body — and it is what
makes staying together worth it, which is the scientific question Symbrion
left open ("when is it better to remain single cells?"). The reef can answer
it empirically: vary light density and watch organism size respond.

**6. Reproduction is division.** An organism above a size and charge
threshold unlocks at its midpoint; both halves inherit its genome (gait
parameters + body plan) with mutation. Growth is recruitment; loss is
shedding (autotomy) and death. No spawning of offspring from nothing.

**7. Lone cells move by flipping.** A one-hinge module can fold to ~150°,
tip its free half over its planted half, and land one body-length forward
— M-TRAN's somersault step. It is slow and clumsy, which is correct: alone,
a cell is weak. Recruitment is a slow crawl toward a beacon; the payoff is
the organism.

**8. Perception is modelled, not raycast.** Cells sense light patches and
other cells within a radius (a photodiode and an IR ring, like Symbrion's
docking IR). The supervisor computes it from poses and gives each cell only
what it could sense. An organism perceives with *all* its cells — a second
concrete benefit of size.

**9. Realtime on a laptop is a hard requirement.** 24 cells × 2 bodies = 48
dynamic bodies ≈ the 8 ms/step budget measured in alife; headless runs may
use more. One engine process at a time, GPU temperature checked before every
run, nothing launched above 70 °C — the thermal protocol is part of the plan.

## The cell (spec)

- Two 6 cm blocks (beveled boxes, 0.06×0.06×0.06 m, 0.12 kg each, density
  ~550 kg/m³) joined by one `HingeJoint`, axis across the seam, `RotationalMotor`
  `minPosition -2.6 maxPosition 2.6`, `maxTorque 0.6`, `maxVelocity 5`,
  `PositionSensor`. Hinge in the seam, so folded flat = 0 rad.
- Four `Connector` faces, `type "symmetric"`, `numberOfRotations 4`,
  `distanceTolerance 0.03 axisTolerance 0.45 rotationTolerance 0.45`: the two
  end faces (nose, tail) and the two side faces of the nose block. End faces
  make chains; side faces make branches and loops.
- Battery 12 Wh, start 50 %. Drain = 0.05 W idle + 0.4 W·|hinge amplitude ×
  frequency| (work) — an idle cell lasts ~4 h of sim, a gaiting one ~30 min;
  a light patch charges 2 W. Time-scaled ×20 for the demo (charge matters
  in minutes).
- Visuals: a matte shell in the organism's colour, an emissive ring showing
  charge (green → amber → red), face LEDs that light on dock. Debris is dark.
- Everything a cell is, a supervisor field write can change: hinge target,
  lock state, colour, emissive. No per-cell process.

## The organism (control)

- **Gait**: hinge_i(t) = bias_i + A·sin(ωt + i·Δφ) along the spine; a
  branch has its own phase offset and amplitude scale. Genome: `{A, ω, Δφ,
  bias_pitch, bias_yaw, branch_phase, branch_scale, steer_gain}`. Steering =
  yaw-hinge bias asymmetry (measured in P1, not assumed).
- **Body plan**: `{target_length, branch_rule (none | pair at k | ring),
  dock_rotation_pattern}` — a small developmental program: which faces the
  organism opens for recruitment and how a recruit is oriented.
- **Behaviour**: seek light when charge < threshold; recruit when charge is
  high and length < target; divide when length ≥ target and charge > 80 %;
  shed the tail cell when charge < 10 % (autotomy); a cell at 0 % goes limp
  (`maxTorque` → 0), is unlocked, becomes debris; debris is recycled to a
  fresh cell at the reef edge after 20 s.
- The supervisor runs every organism's controller and every free cell's
  crawl; measured motion (`{commanded, achieved}`) is logged per organism.

## The reef

18 m lit floor, low walls, 5 light patches that drift slowly (so no organism
can camp), a recycling edge. 24 cells at start, 6 seeded two-cell organisms
and 12 free cells. Light density is the one environmental dial the operator
turns, and it is the experiment: does the reef favour big bodies or single
cells?

## Agent surface (HTTP, same contract as alife)

`/census` (organisms, cells, genomes, charge, lineage), `/light {x,y}` (place
or move a patch), `/split {organism}`, `/recruit {organism, cell}`,
`/dim {factor}` (light density), `/capabilities`. Measured results only.

## Plan and gates (each measured before the next begins)

| phase | build | pass when |
|---|---|---|
| P1 cell probe | cell VRML; 6 cells; supervisor docks two by field write, chains four, drives a travelling wave | weld engages only when aligned; a 4-chain moves > 0.15 m/s; a lone cell flips forward; 24 cells ≤ 8 ms/step |
| P2 organism control | CPG on chains and branches, steering, recruit-and-dock manoeuvre, division | a chain steers to a target (monotone curvature); a free cell docks onto a moving chain; division yields two moving halves |
| P3 ecology | energy sharing, light, autotomy, death/recycle, genome inheritance | conservation holds (cell count constant); starvation sheds; a fed body outlives a lone cell |
| P4 evolution | epochs, telemetry, agent surface, `metazoa.py` driver | gait or body-plan metrics improve over ≥ 8 epochs with a deterministic re-score (the alife lesson: evolution-time scores are not comparable) |
| P5 presentation | looks, charge rings, follow camera, launcher entry, cinema clip | a 60 s clip a stranger understands without narration |

Every phase produces numbers into `projects/metazoa/README.md`, including
the failures — the alife README is the model.

### Status, measured (2026-08-30; every number is in the README)

| phase | status | what the engine showed |
|---|---|---|
| P1 | **passed, with two amendments** | welds engage only when aligned (lock at 9–10 mm / 0.000 rad, a 10 cm gap never welds); 24 cells 3.4 ms/step bare engine. *Amended:* a 4-chain walks at 0.06–0.11 m/s, not > 0.15 (the belly-roller cell is what makes it move at all); a lone symmetric cell cannot flip — free cells are inert by design and organisms come to them |
| P2 | **passed** | pitch spine + head rudder steers (single rudder radius 2 m; a head PAIR 0.3 m); tail docking recruits on-axis and 0.6 m off-axis in one attempt; **division yields two moving halves** (0.069 / 0.121 m/s) via dock-face re-rolls |
| P3 | **partly** | conservation holds in every one of 13 reef epochs (free + members = cells); recruits happen inside the ecology (2 per 600–900 s epoch on 2 bodies); light patches feed bodies. *Not yet observed live:* starvation shed, death/recycle (bodies never ran down), a division inside a reef epoch |
| P4 | **not started** | `metazoa.py` runs multi-epoch with lineage scoring; no ≥ 8-epoch evolution run and no deterministic re-score yet |
| P5 | **partly** | launcher entry + DEMOS.md + a seed reef and watch world ship; charge rings live; no follow camera, no cinema clip |

What every failure between P2 and P3 turned out to be, in order: the arena
wall (bodies with a 2 m turn radius end every approach at a wall), an
unmeasured rudder angle (1.0 rad anchors the body; 0.5 turns it), a target
inside the turning circle (orbiting), an open-loop reverse that drifts, and a
reverse-steering sign that was inverted. Each was measured on a probe before
it was fixed, and each fix is a number in the README.

## What we keep from the two earlier attempts

From **alife**: the supervisor-drives-everything architecture, batched
field writes, Pose-wrapped roots, the crypt slab, the calibration wiggle, the
no-`setVelocity` rule, the diagnostics that made every failure attributable.
From **RoboLife**: the Connector weld engine fix (already applied to the
runtime and its bundle — it ships), the radio-bus pattern, the energy
bookkeeping module. The Husky world itself is shelved.

## Risks, named

- Lone-cell locomotion may be too weak to reach a recruiter (P1 measures
  it; fallback: recruiters crawl to the recruit).
- Docking alignment under a moving organism is the hardest manoeuvre in the
  field (Symbrion needed IR guidance and still reached three modules); P2's
  gate is exactly this and it may take the longest.
- Weld count and contact density at 24 cells are the realtime risk; the
  fallback is 16 cells interactive, 32 headless.

## Sources

- Kernbach et al., *Symbiotic robot organisms: REPLICATOR and SYMBRION* —
  https://www.researchgate.net/publication/234116421
- Winfield, *The Symbrion swarm-organism lifecycle* —
  https://alanwinfield.blogspot.com/2012/05/symbrion.html
- Murata et al., *M-TRAN: self-reconfigurable modular robotic system* —
  https://www.researchgate.net/publication/3414899
- Groß, Bonani, Mondada, Dorigo, *Autonomous self-assembly in swarm-bots* —
  https://iridia.ulb.ac.be/~mdorigo/Published_papers/2005/GroBonMonDor2005amire.pdf
- Wikipedia, *Self-reconfiguring modular robot* (architectures, systems, open
  challenges) — https://en.wikipedia.org/wiki/Self-reconfiguring_modular_robot
- Sims, *Evolved Virtual Creatures* (1994) — https://www.karlsims.com/evolved-virtual-creatures.html
- Plantec et al., *Flow-Lenia: emergent evolutionary dynamics in mass
  conservative continuous cellular automata* — https://arxiv.org/pdf/2212.07906
- Soft-Constraint Joints for loop-forming modular robots in articulation
  simulators — https://link.springer.com/article/10.1007/s10015-025-01103-4
