# Render evidence scripts, 2026-09-12

These scripts produced the committed evidence files
`docs/benchmarks/data/temporal-rendering-2026-09-12.json`,
`object-motion-2026-09-12.json` and `local-shadow-2026-09-12.json`, for commits
`af23053e2`, `37aa0fc84`, `c1c9e2528` and `affa49d62`. They were copied on
2026-10-08 from the ignored `.local-runs/beauty-realism/` working directory,
along with the check outputs they assert against (`*-checks.stdout`), the
commit messages, `shadow-machine.txt` and the beauty-bench A/B reports
(`comparison/`).

The scripts still contain their original hard-coded paths
(`.local-runs/beauty-realism/`, `.local-runs/{temporal,motion,shadow}-pipeline/`,
and installed binaries under `msys64/`). They are a record of how the numbers
were made; they cannot be run unchanged. The raw pipeline outputs they read are
kept in `docs/benchmarks/data/raw/2026-09-12-{temporal,motion,shadow}/`.
