# ARCHITECTURE.md — repo layout and extension recipes

How OmniSim is laid out and where to put new things. The goal is that someone landing in a fresh clone can find any demo, world, agent, or test in one hop from this document.

For the demo and world dictionaries, see [DEMOS.md](DEMOS.md) and [WORLDS.md](WORLDS.md). For the agent entry point, see [AGENTS.md](AGENTS.md).

---

## TL;DR

A **scenario** in OmniSim is composed of four things, in increasing degree of optionality:

1. A **world** (`.wbt`) — what physically exists, where, and which controllers are bound.
2. One or more **controllers** — code that runs inside OmniSim and drives a robot or supervises the scene. Native or Python.
3. Optionally a **bridge** — a long-running controller that exposes an HTTP surface so an external process can drive the scene (the OmniLink agent pattern).
4. Optionally an **agent** — a profile + tools that talks to the bridge, either OmniLink-hosted or local Python.

The repository's job is to keep these four layers cleanly separable so each can be authored, swapped, tested, and demoed independently.

---

## Repository layout

```
omnisim/
├── README.md, AGENTS.md, CHANGELOG.md, CONTRIBUTING.md, PROTOCOL.md, ...
├── DEMOS.md, WORLDS.md, ARCHITECTURE.md   ← top-level indexes
│
├── agents/                       Single home for every OmniLink agent
│   ├── README.md, ROADMAP.md, AGENT_PATTERNS.md, requirements.txt
│   ├── templates/                Profile-only starter kits (picker, roomba, foreman)
│   ├── production/               Full agents with their own runners (5) + shared _lib/
│   ├── benchmarks/               Scripted-vs-agentic head-to-head harness
│   └── bridges/                  Real-robot bridge starter kit (mock-driver stubs)
│
├── projects/                     OmniSim project library (PROTOs, worlds, controllers)
│   ├── appearances/              Shared visual appearances (PROTOs)
│   ├── objects/                  Furniture, plants, vehicles, props (PROTOs)
│   ├── robots/                   URDF + PROTO robots (Clearpath, OmniLink, Unitree, …)
│   ├── default/                  Default world template + helpers
│   ├── languages/                Per-language controller wrappers
│   ├── devices/                  Per-device example controllers
│   ├── policies/                 Policy pipeline — see below
│   ├── robot_combat/             Robot-combat worlds + controllers
│   ├── omni_quest/               Quest scenario worlds
│   └── samples/
│       ├── devices/              Device-by-device pedagogical worlds
│       ├── geometries/           Geometry primitives showcase
│       ├── protogen/             PROTO generator example PROTOs
│       ├── rendering/            PBR, animated skin, Sponza
│       └── demos/
│           ├── controllers/      Shared controller library for every demo
│           ├── protos/           Shared PROTOs (OmniLinkStage, …)
│           └── worlds/           Every demo, grouped by category
│               ├── omnilink_launcher.omniworld         default entry point (stays flat)
│               ├── chat/         15 single-robot natural-language demos  (public count lower: held robots)
│               ├── flagship/     17 multi-robot agent-driven showcases  (public count lower: held robots)
│               ├── showcase/     6 Husky variants / outdoor / fall tests
│               ├── physics/      6 Newton / CUDA / granular stress
│               ├── rendering/    19 camera / wgpu / PBR render-smoke worlds
│               ├── environments/ 3 outdoor scenes (city, desert, forest)
│               ├── dev/          3 in-progress scene previews
│               └── misc/         2 transform / stack tests
│
├── distribution/
│   └── generated_worlds/         Procedurally generated worlds (omniworld + seeds)
│
├── tests/                        OmniSim test suite (layout inherited from upstream Webots)
│   ├── api/, physics/, rendering/, parser/, protos/, cache/, smoke/, …
│   ├── benchmarks/               Perf sweep harness
│   ├── python/omniworld/         omniworld generator unit tests
│   ├── harness/, capture/, sources/
│   └── cuda/                     CUDA particle/granular stress
│
├── docs/                         Long-form docs (developer/, guide/, reference/, showcase/)
├── scripts/                      Dev tooling, harness, capture, cinema, release
├── omnisim/                      Python package (CLI, omniworld, doctor, damage, …)
├── packages/                     Pip-installable sub-packages (omnisim-bridges)
├── src/, include/, lib/, bin/    C/C++ engine sources and build outputs
├── dependencies/                 Third-party deps
└── resources/                    Engine resources (PROTOs, plugins, robot windows)
```

`projects/policies/` is the policy pipeline — Shadowing trainers (`training/`), the skill library
(`skills/`), deploy controllers (`controllers/`), worlds (`worlds/`), and the standalone research
trainers (`research/`). Start at [`projects/policies/training/README.md`](projects/policies/training/README.md).

Adding a new demo is: pick a subfolder, drop in a `.wbt` and a matching controller, add an entry to
`demos.json`, add a row to [DEMOS.md](DEMOS.md). The launcher picks up the new entry on the next
world load.

Worlds are subdivided by category while controllers stay in one shared library — the same
`omnilink_arm_bridge` powers several chat worlds, so duplicating controllers per demo would cost
more than it buys.

---

### Design principles

1. **One canonical home per concern.** Every demo world lives under `projects/samples/demos/worlds/<category>/`. Every agent lives under `agents/`. No parallel trees.
2. **Worlds are categorised; controllers are shared.** A demo is a `.wbt` in a category subfolder plus a controller in the shared `projects/samples/demos/controllers/` library. Any world can bind any controller in that library.
3. **Shared resources stay shared.** Robots, appearances, bounding objects, and reusable PROTOs live in `projects/` and are referenced via portable `omnisim://projects/...` URLs.
4. **The four layers stay separable.** World, controller, bridge, and agent can each be authored, swapped, tested, and demoed independently.

---

## How to add a new demo

1. **Pick a category subfolder** under `projects/samples/demos/worlds/` (e.g. `chat/`, `flagship/`, `physics/`). Create the subfolder if it doesn't exist yet.
2. **Author the world** as `projects/samples/demos/worlds/<category>/<your_demo>.wbt`. Path conventions inside the WBT:
   - `EXTERNPROTO "omnisim://projects/samples/demos/protos/..."` — portable, resolved against `OMNISIM_HOME` (the repo root).
   - `texture "omnisim://..."` — same.
   - `url "..."` on `URDFRobot` — **must be a relative path** (e.g. `../../../../projects/robots/<vendor>/...`); the URDF loader does not understand `omnisim://`. Count `../` carefully based on subfolder depth.
3. **Author the controller(s)** as `projects/samples/demos/controllers/<your_controller>/<your_controller>.py`. OmniSim finds controllers by name from the project root, so any demo can bind any controller in this directory.
4. **Add the demo to the launcher manifest** at [`projects/samples/demos/controllers/omnilink_launcher/demos.json`](projects/samples/demos/controllers/omnilink_launcher/demos.json). One entry: `{id, name, world, blurb}` under the right category.
5. **Add a row to [DEMOS.md](DEMOS.md)** and (if appropriate) [WORLDS.md](WORLDS.md).
6. **If the demo introduces a new agent**, mirror that under `agents/production/<your_agent>/` (or `agents/templates/` for profile-only specialists). See [agents/README.md](agents/README.md).

The launcher picks up the new demo the next time its world is opened — no rebuild, no controller registration step.

---

## How to add a new world

Worlds fall into one of three buckets — pick the right home upfront:

| Bucket | Home | What goes here |
|---|---|---|
| Demo world | `projects/samples/demos/worlds/<category>/` | Anything user-facing or showcased |
| Generated world | `distribution/generated_worlds/` | Output of the `omniworld` generator (always paired with a `.seed.json`) |
| Test world | `tests/<suite>/worlds/` | Regression coverage for engine, parser, physics, etc. |

For shared library content (robots, props, appearances), don't author a world — add a PROTO under `projects/{robots,objects,appearances}/` instead. Worlds reference those PROTOs.

Every new demo / sample / generated / RL world MUST use the canonical lighting recipe — three PROTOs (`OmniSimSky`, `OmniSimSun`, `OmniSimSunMarker`) that reproduce the OmniQuad demo look across every user-facing world. See [`docs/WORLD_RECIPE.md`](docs/WORLD_RECIPE.md) for the copy-pasteable block and the extension rules. Test worlds under `tests/` are exempt.

---

## How to add a new agent

```text
agents/<templates|production>/<your_agent>/
├── README.md
├── profile.json            OmniLink profile (system prompt, mainTask, defaults)
├── omnilink.json           OmniLink workspace pin
├── prompts/                Long-form prompts pulled into profile.json
├── tools/                  Auto-discovered Python tools
├── knowledge/              Static knowledge files
├── long_term_memory/       Memory state (gitignored or committed depending on agent)
└── <your_agent>_agent.py   Thin runner (production only — templates skip this)
```

Pick `templates/` for profile-only specialists that reuse an existing bridge, `production/` for full agents with their own runner / memory / docs.

---

## How to add a new test

Tests already live under `tests/<suite>/worlds/` + `tests/<suite>/controllers/`. The suite layout (`api`, `physics`, `parser`, `protos`, `cache`, `rendering`, `other_api`, `cuda`, `smoke`, `benchmarks`, `default`, `manual_tests`) is the convention inherited from upstream Webots — don't add new top-level test categories without a good reason.

For the omniworld procedural-generation tests, add to [`tests/python/omniworld/`](tests/python/omniworld/). For source-hygiene checks (formatting, license, etc.), add to [`tests/sources/`](tests/sources/).

Driver: [`tests/test_suite.py`](tests/test_suite.py). Smoke subset: [`tests/smoke/run_smoke.py`](tests/smoke/run_smoke.py).
