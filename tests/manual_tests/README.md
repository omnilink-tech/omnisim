# Manual / GUI-only tests

These worlds are **not run by any automated test runner**. Neither
`tests/test_suite.py` (which globs `tests/<group>/worlds/*.wbt`) nor
`tests/test_worlds.py` (which walks `projects/**/*.wbt`) touches this
directory, and the pre-push hook ignores it.

Each world here exercises an interaction that only makes sense with a human
in front of the editor — selecting nodes, pasting PROTOs, editing template
fields, observing reference-model behaviour. Automating them would require
real GUI input, not just a headless simulation step.

## Worlds

| World | What to verify |
|---|---|
| `derived_proto_solid_physics.omniworld` | Physics on a derived PROTO behaves like the parent |
| `insertion_in_nested_parameter.omniworld` | Inserting nodes into a nested PROTO parameter |
| `interaction_with_solid_reference_model.omniworld` | Selecting/moving a Solid reference model — ⛔ **CURRENTLY CRASHES THE ENGINE ON LOAD, see below** |
| `modify_proto_template_field.wbt` | Editing a template field re-regenerates the PROTO |
| `paste_proto_in_def_node.omniworld` | Pasting a PROTO into a DEF node updates references |
| `selection_when_procedural_proto_regeneration.omniworld` | Selection survives procedural regen |
| `transform_proto_parameter.omniworld` | Editing a Transform parameter on a PROTO instance |

## ⛔ Known red: `interaction_with_solid_reference_model.omniworld`

**It does not load at all — the engine dies with a stack overflow before it
opens the world.** Measured 2026-08-16 (machine `9722d23d12a3`, binary
`msys64/mingw64/bin/omnisim-bin.exe`):

```
python -m omnisim run-headless \
  tests/manual_tests/worlds/interaction_with_solid_reference_model.omniworld --until-finalized
[headless] FAIL: simulator exited early with code 3221225725
```

`3221225725` is `0xC00000FD` = `STATUS_STACK_OVERFLOW`, i.e. unbounded
recursion. The log stops after the two startup lines — it never reaches
`world opened`. Control: `derived_proto_solid_physics.omniworld` in this same
directory loads and exits 0 on the same binary in the same session, so this is
the world, not the environment or the runner.

**This is precisely the regression the world exists to catch.** Its own
`WorldInfo.info` says *"To test that no infinite loop occurs loading the world
or interacting with the model — Regression test for issue #7064"*. The infinite
loop is back.

⚠️ **Do not "fix" it by deleting the loop-closing `SolidReference`.** That
`HingeJoint { endPoint SolidReference { solidName "Link_1" } }` at
[`:98`](worlds/interaction_with_solid_reference_model.omniworld#L98) **is the
subject under test** — removing it deletes the regression test. This world is a
*different* failure from the loop-closure-kills-physics class documented in
[docs/reference/solidreference.md](../../docs/reference/solidreference.md): that
one loads fine and then fails at Newton finalize with `Body N has multiple
parents in this articulation`. This one never gets as far as finalize.

Not in any automated lane (see the note at the top of this file), so it takes
nothing red in CI — it just means this row cannot be exercised by hand until the
recursion is found. Root cause not yet investigated.

## When to run

Before a release that touches the scene-tree editor, PROTO regeneration, or
selection handling. Open each world, exercise the listed interaction, and
confirm no crashes or visible regressions.
