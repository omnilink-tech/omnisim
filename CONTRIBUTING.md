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
  3. **The build reads it widely.** The top-level [`Makefile`](Makefile) exports `WEBOTS_HOME` as an alias of `OMNISIM_HOME`, and roughly twenty Makefiles consume it — not just `src/controller/{c,cpp,launcher}/Makefile`, but also `resources/Makefile.include`, the `dependencies/` and `src/{ode,wren,glad}` Makefiles, and the `resources/projects/**` library/plugin Makefiles. A top-level `make` therefore works with `OMNISIM_HOME` set alone (the alias is exported for you); a **standalone** `make` inside a controller or plugin directory does not go through the top-level Makefile, so it needs the variable in its own environment.

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

Say "Webots" **only** when you are factually referring to upstream: the [upstream repository](https://github.com/cyberbotics/webots), upstream's documentation and issue tracker, the file-format syntax and PROTO conventions we inherited, or the legacy compatibility aliases (`webots/…` include paths, the `WEBOTS_HOME` environment variable, the `webots.exe` launcher shim). In those cases "upstream Webots" is the clearest phrasing.

Two failure modes, both worth catching in review:

- **Using "Webots" to mean *our* simulator** — reads as if the rebrand never happened.
- **Crediting OmniSim with work upstream actually did** — e.g. describing the WREN renderer or the PROTO system as an OmniSim invention. This is the more serious one: it is a false provenance claim, and it undermines the attribution we owe Cyberbotics. Attribute honestly; the additions are substantial enough to stand on their own.

See [TRADEMARKS.md](TRADEMARKS.md) for the trademark boundary (we claim the OmniSim/OmniLink marks; we claim **no** rights in Cyberbotics' "Webots" mark).

## Licensing of Your Contribution

OmniSim is licensed under the **Apache License 2.0** (see [LICENSE](LICENSE) and [NOTICE](NOTICE)). By submitting a contribution to this repository, you agree that your contribution is licensed under the same Apache License 2.0 as the rest of the project — this is the standard *inbound = outbound* rule that Apache 2.0 §5 already encodes, so there is no separate Contributor License Agreement (CLA) to sign.

We do, however, use the **Developer Certificate of Origin (DCO)** — the same lightweight mechanism used by the Linux kernel, Docker, GitLab, and many other open-source projects. To certify that you have the right to submit your work under the project's license, please add a `Signed-off-by` line to each commit:

```
Signed-off-by: Your Name <your.email@example.com>
```

You can do this automatically with `git commit -s` (or `git commit --signoff`). The full text of the DCO is at <https://developercertificate.org> — it is a short, plain-English statement that you wrote the code yourself (or have the right to submit it), and that you understand it will be released under the project's open-source license. No paperwork, no account to register.

## Trademarks and Brand

OmniSim's **code** is open under Apache 2.0; the **"OmniSim" and "OmniLink" names, the orb mark, the tagline, and the brand palette** are trademarks of OmniLink and are governed by the [Trademark Policy](TRADEMARKS.md). In short:

- You are free to contribute to OmniSim, fork it, build on it, and ship products that use it — including commercially.
- You may say your product is **"built for OmniSim"**, **"compatible with OmniSim"**, or **"based on OmniSim"** — that is welcome and does not require permission.
- If you redistribute a **modified** fork, please **rename it and replace the OmniSim/OmniLink branding assets**. The code stays Apache 2.0; the brand identifies OmniLink's work.
- **"Webots" and "Cyberbotics" are Cyberbotics Ltd.'s trademarks, not ours.** Apache 2.0 §6 grants no trademark rights, so upstream's code license gave us no license to their marks — and we pass none on to you. Don't brand anything with them, and don't imply Cyberbotics endorses this fork.

This is the same pattern Mozilla uses for Firefox, the Rust Foundation uses for Rust, and Signal uses for the Signal app — open code, protected brand. It lets us welcome you as a contributor while keeping the "OmniSim" name meaningful for users.

## Code of Conduct

Be kind, be specific, assume good faith, and remember that many contributors are working in a second language or across very different time zones. Harassment, personal attacks, and discriminatory language are not welcome in issues, PRs, or any other project space. Maintainers reserve the right to close threads and remove content that violates this.

## Attribution

OmniSim is built on top of the [Webots](https://github.com/cyberbotics/webots) open-source project by Cyberbotics Ltd. Both OmniSim and the upstream Webots it depends on are released under the Apache License 2.0 — please respect the original license and attribution terms when contributing.

---

If anything in this document is unclear, open an issue and ask. We would rather have the conversation than have you guess. Welcome to the project.

— The OmniSim maintainers, on behalf of [OmniLink](https://www.omnilink-agents.com)
