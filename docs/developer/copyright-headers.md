# Copyright headers

This document defines the source-file copyright header convention for OmniSim.
It exists to satisfy Apache License 2.0 §4(a) (preserve copyright notices) and
§4(b) (carry prominent notices stating that you changed files), and to keep
attribution clean across files of different lineages.

## Three categories of files

Every source file in the repository falls into one of three buckets. The
required header differs by bucket.

### 1. Unmodified Webots files

Files copied verbatim from Cyberbotics' upstream Webots repository.

**Rule:** keep the existing Cyberbotics header exactly as-is. Do not edit it,
do not append to it. If you find yourself wanting to modify such a file, it
moves to category 2.

Example (already present in many files under `src/omnisim/`):

```cpp
// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// ...
```

### 2. Modified Webots files

Files that originated in Webots but have been changed for OmniSim.

**Rule:** preserve the Cyberbotics header verbatim, then add a single
modification line directly below it.

```cpp
// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...
// limitations under the License.
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
```

One line is enough — Apache 2.0 §4(b) only requires a "prominent notice" that
the file was changed; it does not require a per-change log. The git log
remains the source of truth for *what* changed.

### 3. New files authored for OmniSim

Files created by OmniLink with no Webots ancestry (most of `agents/production/`,
`scripts/harness/`, new `omniworld` biomes, new bridges, etc.).

**Rule:** use the OmniLink-only header.

```cpp
// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
```

Python/shell equivalent uses `#` comments; otherwise identical text.

## What we do not do

- **No author lines.** Headers carry the entity copyright (Cyberbotics or
  OmniLink), never individual contributor names. Attribution lives in git
  history and the public repo's commit log, which under the Option B release
  model is single-authored per snapshot.
- **No year ranges that extend into the future.** Use the year the file was
  first committed under that copyright; bump on substantive edits in a new
  year, not eagerly.
- **No "All Rights Reserved" wording** — it conflicts with the Apache 2.0
  grant.

## Enforcement

The convention is enforced by [`tests/sources/test_license.py`](../../tests/sources/test_license.py):

```bash
python -m unittest tests.sources.test_license
```

It walks `src/controller/{c,cpp,launcher}`, `src/omnisim`, `projects`,
`include/controller`, `include/plugins`, `scripts`, `packages`, `omnisim` and
`agents` (`*.c *.cpp *.h *.hpp *.py *.java Makefile`; list re-synced with the
test 2026-09-01 — `src/wren` left it with the WREN deletion) and asserts the header sits at the **very start** of the file —
for Python and Makefiles, immediately after an optional `#!/usr/bin/env python[23]`
shebang. A UTF-8 BOM makes the check impossible to satisfy, so source files must not
carry one.

Category 2 (modified Webots) files satisfy it with the Cyberbotics block plus the
modification notice; category 3 (OmniLink-authored) files with the OmniLink-only
block.

**If you write a code generator that emits source files, emit the header too** —
otherwise a regeneration silently strips it and the test goes red. See
`projects/omni_quest/tools/gen_*.py` for a worked example. (The former example here,
`scripts/dev/make_omnisim_header_forwarders.py`, was deleted on 2026-08-16 along with
the `include/controller/*/webots/` shims it generated; the rule it illustrated stands.)
