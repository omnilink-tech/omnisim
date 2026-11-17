# Launch-race measurements, 2026-08-29

These are the raw summaries behind the v8.1.13 CHANGELOG entry for the startup
race (#3): 3 of 7 rounds raced with `--concurrent 2 --stagger 12`, and 0 of 8
when the engine's stdout was a pipe. The `fixed/` runs are the same tests on the
fixed binary. `scripts/dev/launch_race_stress.py` produced them. They were
copied on 2026-10-08 from the ignored `.local-runs/launch_race_stress/`
directory. The per-launch engine logs were not copied.
