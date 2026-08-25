# OmniSim Documentation

This directory holds the documentation for **OmniSim** — a robotics simulator built to be driven by AI coding agents. You talk to it; you don't configure it. Claude Code builds it, OmniLink runs it.

The docs are organised into the following books:

- [`developer/`](developer/) — implementation-facing docs for OmniSim contributors (architecture, build/iteration, performance, OmniWorld procedural generation, validation playbooks). **Start here if you are working on the simulator itself.**
- [`guide/`](guide/) — user guide: installation, getting started, tutorials, controllers, scene tree, sample worlds.
- [`reference/`](reference/) — reference manual: world-file nodes, fields, controller API, supervisor, physics.
- [`benchmarks/`](benchmarks/) — performance measurements, the cross-simulator comparison paper, and the OmniBench campaign reports (the runnable suite lives in `tests/benchmarks/omnibench/`).

Release notes are **not** in this directory: OmniSim's changelog is the root [`CHANGELOG.md`](../CHANGELOG.md).

## OmniSim and its upstream

OmniSim is a fork of [Webots](https://github.com/cyberbotics/webots). The `guide/` and `reference/` books were imported from Webots and are progressively being rebranded and updated as OmniSim evolves. Where you still see "Webots" in technical content, the underlying behaviour generally still applies — OmniSim inherits the world-file format, controller API, and PROTO system. New OmniSim-specific docs (build, runtime, agentic authoring, OmniWorld) live under [`developer/`](developer/).

If you find a doc page that is out of date or still describes Webots-specific assumptions that no longer hold in OmniSim, update it in the same change as the code that diverges.

## Running the docs locally

1. Set the terminal to the `docs` directory:

```sh
cd $OMNISIM_HOME/docs
```

(`$OMNISIM_HOME` points at the OmniSim install root and is the canonical variable — setting it alone is sufficient to build and run. **`$WEBOTS_HOME` is not retired**, but it is now a *build-and-Qt* variable rather than a general one: (1) ⚠️ **the core runtime — libController, the controller package, the launcher — reads `OMNISIM_HOME` and NOTHING ELSE.** This line claimed a `WEBOTS_HOME` fallback until 2026-08-16 and was wrong: a setup exporting only the legacy name is *detected and refused* with a message naming the new variable, not silently honoured ([`system.c:137`](../src/controller/c/system.c#L137), [`omnisim_controller.c:161`](../src/controller/launcher/omnisim_controller.c#L161), [`wb.py:27`](../lib/controller/python/omnisim/wb.py#L27)); (2) the shipped `qt_utils` runtime library reads `OMNISIM_HOME` first and **still falls back to `WEBOTS_HOME` at runtime** ([`StandardPaths.cpp`](../resources/projects/libraries/qt_utils/core/StandardPaths.cpp)); and (3) the **build** exports `WEBOTS_HOME` from the top-level [`Makefile`](../Makefile) as an alias, and **~20 Makefiles consume it**. Write `OMNISIM_HOME` in new code; do not assume `WEBOTS_HOME` has been purged. See [AGENTS.md](../AGENTS.md) §10 for the naming policy and [`scripts/dev/rename_audit.py`](../scripts/dev/rename_audit.py) for the live audit of what stays Webots-named.)

2. Create or update the `index.html` page:

```sh
python local_exporter.py
```

3. Run a simple HTTP server:

```sh
python -m http.server 8000
```

4. Then in a browser, open:

- [http://localhost:8000/?url=&book=guide](http://localhost:8000/?url=&book=guide)
- [http://localhost:8000/?url=&book=reference](http://localhost:8000/?url=&book=reference)

The `developer/` and `benchmarks/` books are plain markdown — read them in your editor or on GitHub directly.

## Running the unit tests

1. Install the `pycodestyle` module:

```sh
pip install pycodestyle
```

2. Run the tests:

```sh
cd $OMNISIM_HOME/docs/tests
python -m unittest discover
```

## Contributing

Contributions are welcome:

1. Fork the repository
2. Make your modifications
3. Open a pull request

When code and docs disagree, update the docs in the same change.
