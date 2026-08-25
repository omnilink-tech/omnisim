# Contributing

Thanks for wanting to build on OmniSim. This project exists because robotics simulation should be accessible, agent-friendly, and shaped by the people who actually use it — and that means you.

OmniSim is **maintained by [OmniLink](https://www.omnilink-agents.com)** and is purpose-built as the simulation environment for the OmniLink agentic AI platform, but it is open source under the Apache License 2.0 and open to everyone. We welcome contributions of every size — from typo fixes to new robots, controllers, worlds, agents, sensor models, renderer improvements, docs, and bug reports.

The following is a set of guidelines for helping you contribute to OmniSim.

## Before You Start

- **If you are an AI coding agent** (Claude Code, Codex, Cursor, or similar) working in this repo, read [AGENTS.md](AGENTS.md) first. It is the canonical entry point with copy-paste commands for build, launch, demo selection, and headless runs.
- **If you are a first-time human contributor**, walk through the [Developer Quickstart](docs/developer/quickstart.md), which covers MSYS2/Qt6 setup, build, and the first demo run end-to-end.

## Required Skills

You don't need to be an expert in robotics or software development to become a contributor.
Depending on your skills, your contribution may address different parts of the OmniSim software:

- **Bug reporting**: [A precise description](../../issues/new?template=bug_report.md) of a reproducible bug is very helpful to us.
- **Technical English writing**: [documentation pages](docs/).
- **Python programming**: [sample simulations](projects/languages/python/controllers), libraries, tools, etc.
- **C/C++ programming**: [source code](src/), sample simulations, libraries, tools, etc.
- **3D modeling**: contribute models of [objects](projects/objects), [devices](projects/devices) or [robots](projects/robots).
- **Procedural world authoring**: add a new biome to [omniworld](docs/developer/omniworld-user-guide.md) — see the [biome cookbook](docs/developer/omniworld-biome-cookbook.md).

In any case, you should have a minimal knowledge of GitHub to fork our repository and create a Pull Request that we will review and hopefully accept.

For C/C++ contributions it is strongly recommended that you install the development environment as explained below.

## Install the Development Environment

* See the [Developer Quickstart](docs/developer/quickstart.md) for the full Windows (MSYS2 + MinGW64) setup. Linux and macOS use the same `make` / `python scripts/dev/omnisim_dev.py build all` flow with the appropriate native toolchain.
* **Note:** `OMNISIM_HOME` is the canonical environment variable for the install root — setting it alone is sufficient to build and run, and it is the name to use in new code. `WEBOTS_HOME` survives as a **legacy alias** kept for compatibility with the upstream Webots project, and it is still read in three places, so do not assume it has been purged:
  1. **Core runtime** (libController, the Python/C/C++ controller package, the controller launcher) reads **only** `OMNISIM_HOME` — no `WEBOTS_HOME` fallback.
  2. **One shipped runtime library still reads it**: [`resources/projects/libraries/qt_utils/core/StandardPaths.cpp`](resources/projects/libraries/qt_utils/core/StandardPaths.cpp) resolves the install root at **runtime** to build the Qt plugin + icon search paths used by robot windows. It prefers `OMNISIM_HOME` and falls back to `WEBOTS_HOME`.
  3. **The build reads it widely.** The top-level [`Makefile`](Makefile) exports `WEBOTS_HOME` as an alias of `OMNISIM_HOME`, and roughly twenty Makefiles consume it — not just `src/controller/{c,cpp,launcher}/Makefile`, but also `resources/Makefile.include`, the `dependencies/` and `src/glad` Makefiles, and the `resources/projects/**` library/plugin Makefiles. (Older revisions of this list also named `src/ode` and `src/wren`; both directories are gone — ODE with its backend on 2026-08-08, commit `bdc02139`, and WREN with the renderer on 2026-08-23, commit `976b9449d`.) A top-level `make` therefore works with `OMNISIM_HOME` set alone (the alias is exported for you); a **standalone** `make` inside a controller or plugin directory does not go through the top-level Makefile, so it needs the variable in its own environment.

  The bundled `build_omni.bat` and `launch.bat` set both from their own location, so on Windows you usually do not need to export anything manually. See [AGENTS.md §10](AGENTS.md) for the same rule stated for agents.

## Create a Pull Request

1. Fork the repository
2. Create a branch in your fork
3. Pull the branch as a pull request targeting `main`
4. Wait for review of your pull request.

## Development Guidelines

* Match the surrounding code. The C/C++ style is machine-enforced by [`.clang-format`](.clang-format) and the Python style by [`.flake8`](.flake8) — run them rather than reading a style guide. (The conventions were originally inherited from the [upstream Webots coding style](https://github.com/cyberbotics/webots/wiki/Coding-Style/), which is still a useful reference for the older parts of `src/`.)
* Avoid committing files that exist elsewhere. Instead you should link to the source of these files.
* Avoid committing files that can be re-created from other files using a Makefile, a script or a compiler.

### Naming discipline: it's OmniSim

This project is **OmniSim** — its own product, with substantial additions over the Webots engine it forked from. When you write user-facing text (docs, log messages, GUI strings, error text, comments), **call the simulator OmniSim**.

Say "Webots" **only** when you are factually referring to upstream: the [upstream repository](https://github.com/cyberbotics/webots), upstream's documentation and issue tracker, the file-format syntax and PROTO conventions we inherited, or a surviving legacy compatibility alias. In those cases "upstream Webots" is the clearest phrasing.

⚠️ **The set of surviving aliases shrank on 2026-08-16 — check before you cite one.** Still live: the `webots.exe` / `webotsw.exe` launcher shims (built as byte-identical copies of `omnisim.exe` / `omnisimw.exe`, [`src/omnisim/Makefile`](src/omnisim/Makefile) `LAUNCHER_LEGACY`), `WEBOTS_HOME` as a **build** variable (below) and as the `qt_utils` runtime fallback, and the `#VRML_SIM` world/PROTO header, which the tokenizer accepts forever ([`OmTokenizer.cpp:156`](src/omnisim/vrml/OmTokenizer.cpp#L156)) but which nothing should **write**. **Gone, so do not describe them as aliases:** the `webots/…` include paths (all 91 forwarder headers under `include/controller/{c,cpp}/webots/` were deleted — that directory no longer exists), the `controller` Python module, `namespace webots`, and the `WEBOTS_*` engine↔controller runtime variables, which are now detected and *refused* with a named message rather than honoured ([`robot.c:158`](src/controller/c/robot.c#L158), [`omnisim_controller.c:157`](src/controller/launcher/omnisim_controller.c#L157), [`wb.py:22`](lib/controller/python/omnisim/wb.py#L22)).

Two failure modes, both worth catching in review:

- **Using "Webots" to mean *our* simulator** — reads as if the rebrand never happened.
- **Crediting OmniSim with work upstream actually did** — e.g. describing the WREN renderer or the PROTO system as an OmniSim invention. This is the more serious one: it is a false provenance claim, and it undermines the attribution we owe Cyberbotics. Attribute honestly; the additions are substantial enough to stand on their own.

See [TRADEMARKS.md](TRADEMARKS.md) for the trademark boundary (we claim the OmniSim/OmniLink marks; we claim **no** rights in Cyberbotics' "Webots" mark).

## Using AI Tools

**AI is welcome here.** It would be dishonest to pretend otherwise: [AGENTS.md](AGENTS.md) is written *for* coding agents, most of this engine was written with an AI agent under human direction, and the commit history says so. So this is not an anti-AI policy. It is a bar, and the bar is the same one we hold ourselves to.

The rules below are what we ask of a contribution, whoever or whatever helped produce it.

**You are the author, and you are accountable.** If you cannot explain what your change does, and why it is correct, without going back to the tool — do not send it. Reviewer time is the scarcest thing this project has, and a plausible change nobody understands spends it for nothing.

**Disclose non-trivial use.** A commit trailer is enough. This repository uses `Co-Authored-By:`, which is what our own tooling emits; other projects require `Assisted-by:` and a few (Kubernetes, Homebrew) forbid AI commit trailers entirely, so check before you carry our convention somewhere else. You do not need to disclose spelling and grammar help, or a tool that only helped you *read* the code.

**The sign-off is yours alone.** An AI tool must never add a `Signed-off-by` line. The DCO is a legal certification that only a person can make, and the person making it is taking responsibility for the whole change — including the parts a tool wrote. (See [Licensing of Your Contribution](#licensing-of-your-contribution) below.)

**Answer review comments yourself.** Reviewers are trying to reach *you*. Pasting a reviewer's question into a model and pasting the answer back wastes the exchange; if we wanted the model's opinion we could have asked it directly.

**Do not point an autonomous agent at this repository.** Pull requests and issues should be chosen, understood and opened by a human. We will close contributions that appear to have been opened by an agent running unattended.

### The rule that is specific to this project: measure it

This one matters more here than almost anywhere else, because of how OmniSim fails.

A world can load, step, and log nothing bad while being completely wrong. A body free-falling forever through a floor with no `boundingObject` prints `0 errors, 0 warnings … PASS`, byte-identically to a working world. A gripper can report contact while the part is resting on a tray. A robot can be registered, rendered, and listed in the scene tree while the physics solver has never seen it. Confident, plausible, and untrue is this codebase's *characteristic* failure mode — and it is exactly the failure mode language models are best at producing.

So: **an AI-assisted change to physics, controllers or worlds needs a measurement, not a screenshot and not a passing exit code.** State the number, name the machine it came from, and say what you compared it against. `python -m omnisim run-headless <world> --duration 10 --fail-on-runaway` is the cheapest honest check for a physics claim; `--until-finalized` is the cheapest one for a load claim. If you could not measure something, say that plainly instead of implying it works — "unverified" is a perfectly good thing to write in a pull request, and it is worth far more to a reviewer than a confident sentence that turns out to be wrong.

This is the same standard the benchmark documentation holds itself to, and it is the reason those numbers can be trusted. It applies to everyone equally; the tooling just makes it easier to forget.

## Licensing of Your Contribution

OmniSim is licensed under the **Apache License 2.0** (see [LICENSE](LICENSE) and [NOTICE](NOTICE)). By submitting a contribution to this repository, you agree that your contribution is licensed under the same Apache License 2.0 as the rest of the project — this is the standard *inbound = outbound* rule that Apache 2.0 §5 already encodes, so there is no separate Contributor License Agreement (CLA) to sign.

We do, however, use the **Developer Certificate of Origin (DCO)** — the same lightweight mechanism used by the Linux kernel, Docker, GitLab, and many other open-source projects. To certify that you have the right to submit your work under the project's license, please add a `Signed-off-by` line to each commit:

```
Signed-off-by: Your Name <your.email@example.com>
```

You can do this automatically with `git commit -s` (or `git commit --signoff`). The full text of the DCO is at <https://developercertificate.org> — it is a short, plain-English statement that you wrote the code yourself (or have the right to submit it), and that you understand it will be released under the project's open-source license. No paperwork, no account to register.

Sign-off is currently checked by a maintainer at review time rather than by a bot, so a PR without it will not be rejected automatically — you will just be asked to add it. `git rebase --signoff <base>` fixes a branch retroactively.

### The licence gates run on every push, and they are strict

Three checks in [`.github/workflows/licence-provenance.yml`](.github/workflows/licence-provenance.yml) guard the project's ability to be redistributed at all. They are fast (about ten seconds) and they fail the build, so it is worth knowing what they want **before** you open a PR:

| Check | What it requires |
|---|---|
| [`tests/sources/test_license.py`](tests/sources/test_license.py) | Every source file you add carries the Apache-2.0 header used throughout the tree. Its exemption baseline `KNOWN_MISSING_LICENSE_HEADERS` is **empty and must stay empty** — an entry there is debt, not permission. |
| [`tests/sources/test_asset_provenance.py`](tests/sources/test_asset_provenance.py) | Every binary asset you add (mesh, texture, font, dataset, weights) has a licence or provenance file at or above it saying where it came from. |
| [`tests/sources/test_licence_pointers.py`](tests/sources/test_licence_pointers.py) | Every repo path cited by [`NOTICE`](NOTICE) or [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) resolves — an attribution that points at a deleted file has silently stopped being made. |

Run them locally with `python -m pytest tests/sources/test_license.py tests/sources/test_asset_provenance.py tests/sources/test_licence_pointers.py`, or any of them with `--report` for a human-readable inventory. ⚠️ **Do not run a bare `pytest` at the repository root** — `tests/test_worlds.py` re-saves every world through the engine and will rewrite your working tree.

**If you are contributing a third-party asset, one question decides it: is the licensor the design owner?** A vendor's ROS package being BSD-3 does not cover another manufacturer's CAD sitting inside it. That single test is what found the Boston Dynamics, Robotiq and Orbbec geometry this project had to remove. See [`docs/developer/asset-provenance.md`](docs/developer/asset-provenance.md).

## Trademarks and Brand

OmniSim's **code** is open under Apache 2.0; the **"OmniSim" and "OmniLink" names, the orb mark, the tagline, and the brand palette** are trademarks of OmniLink and are governed by the [Trademark Policy](TRADEMARKS.md). In short:

- You are free to contribute to OmniSim, fork it, build on it, and ship products that use it — including commercially.
- You may say your product is **"built for OmniSim"**, **"compatible with OmniSim"**, or **"based on OmniSim"** — that is welcome and does not require permission.
- If you redistribute a **modified** fork, please **rename it and replace the OmniSim/OmniLink branding assets**. The code stays Apache 2.0; the brand identifies OmniLink's work. Note that the artwork under [`resources/branding/`](resources/branding/) is reserved by **copyright** as well as trademark — see [`resources/branding/LICENSE`](resources/branding/LICENSE) — so it is not yours to ship even unaltered as a fork's identity. Using it *unmodified* to refer to OmniSim itself is fine and needs no permission.
- **"Webots" and "Cyberbotics" are Cyberbotics Ltd.'s trademarks, not ours.** Apache 2.0 §6 grants no trademark rights, so upstream's code license gave us no license to their marks — and we pass none on to you. Don't brand anything with them, and don't imply Cyberbotics endorses this fork.

This is the same pattern Mozilla uses for Firefox, the Rust Foundation uses for Rust, and Signal uses for the Signal app — open code, protected brand. It lets us welcome you as a contributor while keeping the "OmniSim" name meaningful for users.

## Code of Conduct

The project's full Code of Conduct is [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md), and it is the governing document — including its reporting contact. The short version: be kind, be specific, assume good faith, and remember that many contributors are working in a second language or across very different time zones. Harassment, personal attacks, and discriminatory language are not welcome in issues, PRs, or any other project space. Maintainers reserve the right to close threads and remove content that violates this.

## Attribution

OmniSim is built on top of the [Webots](https://github.com/cyberbotics/webots) open-source project by Cyberbotics Ltd. Both OmniSim and the upstream Webots it depends on are released under the Apache License 2.0 — please respect the original license and attribution terms when contributing.

---

If anything in this document is unclear, open an issue and ask. We would rather have the conversation than have you guess. Welcome to the project.

— The OmniSim maintainers, on behalf of [OmniLink](https://www.omnilink-agents.com)
