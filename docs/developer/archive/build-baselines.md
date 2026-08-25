# OmniSim Build Time Baselines

Measured 2026-04-11 on Windows 11, AMD/Intel laptop, NVIDIA RTX 3060, MSYS2 MinGW64, GCC 15.2, -j8.

## Core simulator (`src/omnisim`)

| Scenario | Time |
|----------|------|
| Clean rebuild (from scratch, -j8) | ~13 minutes (783s) |
| Single-file change + relink | ~35-49s |
| No-op build (nothing changed) | ~6s |

## Renderer (`src/wren`)

| Scenario | Time |
|----------|------|
| No-op build | ~5s |

## With ccache (repeat single-file rebuild)

| Scenario | Time |
|----------|------|
| First compile (cache miss) | ~18s |
| Repeat compile (cache hit) | ~8s |

## Notes

- Linking is the dominant cost for single-file changes (~30s of the 35-49s total).
- ccache reduces compile time but does not help with linking.
- Clean rebuilds populate the ccache, so a subsequent clean rebuild is faster.
- The no-op time (~6s) is spent evaluating Makefile dependencies and moc checks.
- Future link-time improvements (splitting into libraries) would reduce single-file rebuild time significantly.
