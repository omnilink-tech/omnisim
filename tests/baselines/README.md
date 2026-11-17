# tests/baselines

Frozen reference outputs used to detect Webots-name regressions in the
post-rebrand steady state. The audit counts must never grow vs. the
baseline; see [`scripts/dev/rename_audit.py`](../../scripts/dev/rename_audit.py)
and [AGENTS.md §0](../../AGENTS.md) for the naming policy.

| File | Source command | Asserted by |
|---|---|---|
| `rename_audit_baseline.json` | `py -3 scripts/dev/rename_audit.py --json` | [`tests/sources/test_naming.py`](../sources/test_naming.py) -- counts must not grow |
| `headless_omnilink_husky.txt` | `py -3 scripts/dev/headless_runner.py projects/samples/demos/worlds/chat/omnilink_husky.omniworld --duration 6` | manual diff per phase ("0 errors, 0 warnings" must stay) |
| `headless_omnilink_launcher.txt` | same, world = `projects/samples/demos/worlds/omnilink_launcher.omniworld` | same |

## Regenerating after an intentional rename

Any deliberate Webots-name reduction (a path rename, a header forwarder
removal, etc.) should lower the corresponding audit count. Re-snapshot:

```bash
py -3 scripts/dev/rename_audit.py --json > tests/baselines/rename_audit_baseline.json
```

The headless `.txt` snapshots are only "0 errors, 0 warnings" sentinels.

## Fixed during the rebrand cleanup

- ~~`warehouse_logistics.wbt` exits with `0xC0000005` because its controllers
  live in non-default search paths.~~ Moot: the world was removed from the
  tree on 2026-07-19 along with the Warehouse Foreman / Picker demo.

- ~~`mobile_robots_showcase.wbt` fails to load TIAGo / E-puck `URDFRobot`s.~~
  Fixed by dropping the two broken blocks (and the corresponding bullet
  lines from the world's header comment). The remaining six URDFRobots
  (OmniBot, CubeBot, TurtleBot3 Burger / Waffle Pi, Husky, Jackal) all
  exist in-repo and load cleanly. World now reports 0 errors, ~9 warnings
  (xacro placeholder strip + dense mesh, all pre-existing URDF quality).

- ~~`test_suite.py`'s `cache` group crashes on a missing `cyberbotics/webots`
  git remote~~. Fixed in `tests/cache/cache_environment.py`: best-effort
  upstream-URL discovery now flips `CACHE_TESTS_ENABLED` to False when the
  remote is absent instead of raising `StopIteration` at module import. The
  smoke / api / parser / etc. groups now run without that import-time
  crash blocking them. (The cache group itself is correctly skipped on a
  fork that doesn't track upstream Webots.)
