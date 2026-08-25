# World reload cache

OmniSim keeps expensive **immutable** load products alive across same-process
world reloads.  The live node and physics tree is still rebuilt in full.

## Reused work

- Triangle tessellation is keyed by the existing authored-geometry content hash.
  Zero-user entries remain in an LRU pool after world teardown.  The default is
  128 unused meshes and `OMNISIM_TRIANGLE_MESH_CACHE_SIZE` changes that bound.
- Decoded `ImageTexture` pixels are keyed by canonical path, file size, mtime,
  and texture-quality preference.  `QImage` implicit sharing gives each node a
  safe handle while the process cache keeps the decoded storage.  The default
  bound is 256 MiB; set `OMNISIM_DECODED_TEXTURE_CACHE_MB` to change it or `0`
  to disable retention.
- Parsed immutable PROTO models are SHA-256 fingerprinted.  Local PROTOs no
  longer get discarded merely because a reload occurred.  A changed file, a
  missing file, or a model which left the session releases the manager's cache
  reference before the new world parse.

Texture GPU-cache identities use the same file metadata key, so an edited file
cannot alias an older WREN texture which is still referenced elsewhere.

## Correctness boundary

This is an incremental **asset/parse** reload, not live scene-tree patching.
OmniSim still tears down controllers, nodes, rendering instances, and physics
bodies, then builds the new world through the established load path.  Structural
changes therefore retain the existing full-reload fallback automatically.

Retaining live physics bodies, controller processes, or mutable node instances
would require a transaction API with rollback and stable identity semantics.
That work is deliberately deferred; pointer grafting during teardown is not a
safe substitute.

## Measurement protocol

Measure cold and warm loads in the **same harness process** so the second load
can use process caches:

1. Start `python -m omnisim harness` once.
2. `POST /world/load` with an asset-heavy world and record completion latency.
3. Load the same path again without restarting the harness and record latency.
4. Touch one referenced texture and one local PROTO in separate runs.  Each warm
   run must load the changed content, while untouched resources remain reusable.
5. Repeat at least five times and report median cold, median warm, and world
   asset counts.  Do not include Newton's one-time process initialization in the
   asset-cache delta; either preload it or report it separately.

The simulator-free contract test is `tests/test_reload_asset_caches.py`.  A full
benchmark needs a rebuilt GUI binary and a graphics-capable session; it is not a
credible measurement against a pre-change binary.
