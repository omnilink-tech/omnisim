# Provenance — `omnisim.ico`

**Status: RESOLVED.** `src/omnisim/gui/omnisim.ico` is the OmniSim / OmniLink
brand mark, © OmniLink.

This file exists because `scripts/release/publish_snapshot.sh` publishes a
squashed single commit, so git history does not travel to the public repository
— and because this `.ico` is the only binary in an otherwise all-source
directory, which makes it precisely the kind of file a licence audit walks past.

## What it is

The Windows application icon, referenced from `omnisim.rc` and compiled into
`omnisim-bin.exe`.

**It is a byte-identical copy of a file that already carries a licence record.**
Verified 2026-08-24 with `md5sum`:

| md5 | path |
|---|---|
| `fff029dc38c575156fc3091f224b68e6` | `src/omnisim/gui/omnisim.ico` |
| `fff029dc38c575156fc3091f224b68e6` | `resources/icons/core/omnisim_doc.ico` |

So its terms are whatever
[`resources/icons/core/license.txt`](../../../resources/icons/core/license.txt)
states for `omnisim_doc.ico`, and that file is the record: *"OmniSim / OmniLink
brand marks. (c) OmniLink. The code is Apache-2.0; the marks are trademarks
governed by TRADEMARKS.md — you may fork the code, but rename and replace the
branding if you ship a modified fork."*

## History — the bytes are ours, the path is inherited

`git log --follow` traces this file back to `resources/icons/core/webots_doc.ico`
in the squashed Webots import (`0db6a18a7`, 2026-04-11). **The path is inherited;
the content is not.** The icon's bytes were replaced with the OmniLink
dot-sphere orb in `a0859d4d7` (2026-04-30, *"branding: OmniLink dot-sphere orb
as canonical OmniSim identity"*), and the file was renamed in `88f98e169`. No
Cyberbotics artwork remains in it.

This is the general trap with `--follow` in this repository: it reports the
squashed import as the origin of anything whose *path* reaches back there, even
when every byte has since been replaced. The md5 identity above is the evidence
that does not depend on history surviving a squash.

## Do not "deduplicate" these two files

They are consumed by different build stages: `omnisim.rc` compiles this copy
into the executable's resource section, while `resources/icons/core/` is staged
into the installed tree. A symlink or a build-time copy would work on Linux and
break the MSYS2/Windows resource compile, which is the only place this file is
read.
