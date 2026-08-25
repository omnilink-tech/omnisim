# OmniSim Asset Pipeline and World Quality

This guide covers a simple but important truth:

asset behavior is simulator behavior.

Startup time, renderer cost, benchmark noise, and overall product quality are all affected by how worlds reference textures, PROTOs, and remote content.

## Why This Matters

In the current codebase:

- world loading can trigger asset download work
- the download manager pauses simulation while downloads are active
- image textures are loaded, converted, scaled, cached, and uploaded during normal operation
- world tests explicitly check for warnings and uncached assets

That means asset choices directly affect both performance and reliability.

## Current Asset Path

### Remote asset retrieval

`OmDownloadManager` creates downloaders, pauses the simulation when downloads start, and resumes only when all tracked downloads complete.

Implication:

- remote assets can block progress in ways that are visible to users and benchmarks
- startup and reset time can depend on cache state

### Texture loading

`OmImageTexture` currently does the following in the normal path:

- resolve URL or file path
- load image data through `QImageReader`
- detect size and apply texture-quality scaling
- convert to `QImage::Format_ARGB32` when needed
- create or reuse a WREN texture
- track a CPU-side `QImage` in a local cache map

Implication:

- texture size and format choices matter
- worlds with many large unique textures are expensive both on CPU and GPU

### Cache coupling

The texture pipeline uses both WREN-side texture caching and a local `QImage` map for CPU-side access such as picking.

Implication:

- texture lifetime is more complicated than “loaded once”
- destroy/recreate patterns can still cost meaningful time

## World Quality Rules

### Rule 1: Benchmark worlds must be local-asset worlds

Do not use remote assets in:

- benchmark worlds
- smoke worlds
- shipped core validation worlds

Why:

- cache state changes results
- network failures become fake engine failures
- startup metrics become less useful

### Rule 2: Shipped sample worlds should minimize surprise downloads

If a world is intended as a core experience:

- prefer local assets
- document remote dependencies clearly if they are unavoidable
- keep the first-run experience predictable

### Rule 3: Texture size should match real visual need

Good defaults:

- use the smallest texture that still looks correct at the camera distances used in the scenario
- avoid very large unique textures for small props
- do not rely on runtime downscaling to fix oversized assets

### Rule 4: Use alpha only where it matters

Textures with alpha influence pipeline decisions and blending behavior. Do not pay for transparency unless the asset truly needs it.

### Rule 5: Keep material variety under control

Too many unique materials and textures increase:

- load cost
- cache pressure
- texture upload cost
- render-state churn

## Immediate Improvement Opportunities

### A. Remove remote assets from validation scenarios

This is the fastest content-level win available.

Do this for:

- smoke worlds
- benchmark worlds
- future CI performance worlds

### B. Add an asset-quality checklist to world review

When adding or modifying worlds, review:

- remote vs local assets
- texture dimensions
- material count
- collision complexity
- expected contact density

### C. Instrument asset load cost separately

Current load and render numbers mix asset work into broader buckets. Add better visibility into:

- time spent waiting for remote assets
- time spent decoding images
- time spent uploading textures

### D. Normalize “world quality” as part of performance work

Use tests and docs to keep worlds:

- warning-light
- local-asset where practical
- stable across machines

## Texture Guidelines For Contributors

- avoid adding very large textures unless the visual payoff is obvious
- prefer reusing existing materials when possible
- treat `OpenGL/textureQuality` as a fallback, not a substitute for disciplined assets
- remember that `OmImageTexture` may still convert and scale on load, so oversized source assets still cost CPU time

## PROTO and Asset Guidelines

- keep PROTO dependencies local for core scenarios when possible
- if a PROTO depends on remote content, do not use it in benchmark-critical worlds
- be careful when introducing new EXTERNPROTO or remote texture dependencies into widely used example worlds

## Tests That Already Help

`tests/test_worlds.py` already checks:

- unwanted warnings during world loading
- worlds that reference non-cached assets

Use that as a product-quality baseline, not just a test detail.

## Asset Review Checklist

For any new or updated world:

- are all critical assets local?
- does the world load without warnings?
- are texture dimensions proportional to actual on-screen size?
- does the world behave the same with a warm cache and a cold cache?
- is it suitable for smoke, benchmark, sample/demo, or only for a feature-specific test?

## What We Want Later

Later, a better asset pipeline should provide:

- clearer separation between core world readiness and optional remote retrieval
- better asset-load metrics
- stricter benchmark-world rules
- easier prefetch or packaging workflows for core content

That is how the simulator becomes faster, more stable, and easier to evaluate over time.
