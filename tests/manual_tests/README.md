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
| `derived_proto_solid_physics.wbt` | Physics on a derived PROTO behaves like the parent |
| `insertion_in_nested_parameter.wbt` | Inserting nodes into a nested PROTO parameter |
| `interaction_with_solid_reference_model.wbt` | Selecting/moving a Solid reference model |
| `modify_proto_template_field.wbt` | Editing a template field re-regenerates the PROTO |
| `paste_proto_in_def_node.wbt` | Pasting a PROTO into a DEF node updates references |
| `selection_when_procedural_proto_regeneration.wbt` | Selection survives procedural regen |
| `transform_proto_parameter.wbt` | Editing a Transform parameter on a PROTO instance |

## When to run

Before a release that touches the scene-tree editor, PROTO regeneration, or
selection handling. Open each world, exercise the listed interaction, and
confirm no crashes or visible regressions.
