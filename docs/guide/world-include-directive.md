# `#include` for world files

OmniSim adds a C-preprocessor-style `#include "path"` directive to
world (`.wbt`) files so authors can compose worlds out of reusable
fragments instead of copy-pasting hundreds of lines.

> This is an OmniSim extension. Worlds that use `#include` are not
> portable to upstream Webots without the preprocessing applied first.

## Syntax

```text
#include "relative/path/from/this/world.wbt"
#include "/absolute/path/to/world.wbt"
#include "omnisim://projects/objects/.../scene.wbt"
```

The directive must appear on its own line, anchored to the left margin
(leading spaces / tabs are OK). It can sit anywhere in the world body
— before, between, or after `EXTERNPROTO` declarations, in the middle
of nodes, etc.

## What gets spliced

The contents of the included file are inserted verbatim at the
`#include` line, with two adjustments:

1. The `#VRML_SIM ...utf8` header of the included file is stripped, so
   the result has a single header (from the top-level world).
2. The splice is bracketed with comment markers so a parse error
   shows you which file the offending line came from:

   ```text
   # >>> begin omnisim #include "child.wbt" (from child.wbt)
   ...the included content...
   # <<< end omnisim #include "child.wbt"
   ```

`EXTERNPROTO` declarations carried by an included file are spliced
along with everything else and merged into the includer's PROTO
namespace by the existing manager — duplicates are deduped by PROTO
name. Singleton nodes (`WorldInfo`, `Viewpoint`) are **not** auto-
deduped in v1; if the includer and includee both declare one, the
parser will complain. The convention is that includers own the
singletons and includes are environment fragments.

## Recursion

Nested includes work — the included file can `#include` other files
relative to its own path. The expander tracks already-included
canonical paths in a visited set; circular includes are reported as a
parse error and skipped (a comment `# omnisim: skipped circular ...`
is left in place so you can see where). Recursion depth is capped at
16 as a safety net.

## When **not** to use it

- Don't use it to share controller logic — that's what Python imports
  are for.
- Don't use it for PROTOs — the existing `EXTERNPROTO` mechanism
  already gives you reusable nodes with parameters and encapsulation.
- Don't use it to merge two combat scenarios — singletons collide.
  Use it to compose a base environment + a roster fragment + a
  director fragment, etc.

## Example

A combat world that wants to reuse a forest battlefield:

```text
#VRML_SIM R2025a utf8

WorldInfo { basicTimeStep 4 title "Forest battle" }
Viewpoint {
  orientation -0.5773 0.5773 0.5773 2.0944
  position 0 0 32
}

# Pull in the standard outdoor-forest environment (trees, terrain,
# fog, sun, sky). It carries its own EXTERNPROTO declarations for
# Oak / Pine / Sassafras / etc.
#include "omnisim://projects/samples/demos/worlds/environments/forest.wbt"

# Now add the combat-specific roster + director.
DEF RED Robot { ... }
DEF BLUE Robot { ... }
DEF DIRECTOR Robot { name "damage_director" ... }
```

When the world is opened the tokenizer expands the include first, so
the parser sees one continuous VRML stream and never knows two files
were involved.

## Working example

[`projects/robot_combat/orc/worlds/include_example/world.wbt`](../../projects/robot_combat/orc/worlds/include_example/world.wbt)
is a minimal world that exercises the feature — it declares
`WorldInfo` + `Viewpoint`, then `#include`s a sibling
`_base_env.wbt` that supplies sky + sun + floor.

## Implementation note

The expansion is done in `WbTokenizer::expandWorldIncludes()` in
[`src/omnisim/vrml/WbTokenizer.cpp`](../../src/omnisim/vrml/WbTokenizer.cpp).
It runs on the raw `QByteArray` contents read from the world file
**before** tokenization, alongside the existing `URDFRobot` block
expansion. PROTOs and other file types are not preprocessed for
`#include`.
