# `projects/_archive/`

Holding pen for worlds, controllers, and demos that we want to keep on
disk but don't want surfacing as live samples — typically:

- Experiments and one-off scenes that informed a decision but aren't
  meant to be opened by users.
- Film / marketing / video shoots: camera rigs, alternate angles,
  `*_topdown.wbt`, `*_mkt.wbt`, fleet-cam capture worlds.
- Variants we kept "just in case" after replacing a scene with a newer
  version (deprecated showcase cuts, old damage-arena drive loops,
  early CUDA proofs of `*_head_on.wbt`, etc.).
- Anything you'd otherwise be tempted to `git rm` but want browsable
  history of without paying the doc/index cost.

This directory is **not** built, indexed in [WORLDS.md](../../WORLDS.md),
or referenced by `projects/samples/Makefile`. Adding a world here
silently removes it from launcher menus and demo index pages.

## Conventions

- Mirror the source path when it helps: e.g. a showcase world archived
  from `projects/samples/demos/worlds/showcase/foo.wbt` goes to
  `_archive/samples/demos/worlds/showcase/foo.wbt`.
- Keep companion assets (`*.wbproj`, custom controllers, PROTOs) with
  the world so it remains openable from in-place.
- Drop a one-line note at the top of each archived world's `WorldInfo
  info` field explaining *why* it was archived and what replaced it,
  if anything.

## When to delete instead of archive

- Build artifacts, `.exe`s, generated caches — those go in
  `.gitignore`, not here.
- Anything whose source-of-truth is the git history itself (a one-line
  edit you reverted, a debug log) — `git log` is the archive.
- Anything that was never opened by anyone but the author and isn't
  reachable from a doc, ticket, or video.

## Retrieval

Move a world back out of `_archive/` if it earns a new use. Update its
`WorldInfo info` to drop the archival note. No special process — it's
just a `git mv`.
