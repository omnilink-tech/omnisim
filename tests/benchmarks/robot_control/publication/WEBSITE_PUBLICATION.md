# Robotics benchmark website publication — 2026-09-24

Published at the owner's explicit request. No additional model evaluation was
run for publication. The next capability milestone is recorded in
[`../NEXT_STEP.md`](../NEXT_STEP.md).

## Live pages

- Homepage: https://www.omnilink-agents.com/#robotics-benchmark
- Full results: https://www.omnilink-agents.com/benchmarks/robotics-control/index.html
- Evidence: https://www.omnilink-agents.com/benchmarks/robotics-control/evidence.zip

The homepage replaces the prior six-instruction demo cost and unpublished
Gemini benchmark draft with the completed cached Opus comparison. Customer
initial labels incorrectly used OmniSim for the evaluated control integration;
the owner clarified on 2026-09-25 that OmniLink was intended. Original evidence
identifiers are unchanged and mapped in results.json. All six configurations
remain visible, including the two
component-sharing controls that tie completion and slightly undercut cost.

Headline: 33/33 completions; approximately 45% lower all-attempt model cost per
successful task than the tested plain model loop. No broad capability or
market-wide leadership claim. The page explains the development split, single
repeat, shared conditions, assisted grasp, sampled unwanted-motion criteria,
excluded costs, historical campaigns and current live-reproduction limitation.

## Validation

- Production frontend build passed; 78 existing routing/SEO tests passed.
- Desktop and mobile presentation checked in the browser; no page overflow
  or broken images in the tested views.
- Every displayed results-table cell checked against the audited JSON.
- Homepage content outside the replaced section, benchmark stylesheet link
  and footer link is unchanged from the previously deployed homepage.
- All 13 deployed static files were downloaded from the public domain and
  matched the prepared SHA-256 hashes.
- The public evidence ZIP was extracted into a fresh directory and verified
  using `python -I -S verify_archive.py`: 1,143 file hashes, 198 grades, complete
  coverage and no accounting mismatches.
- Durable verification output:
  `LIVE_PUBLICATION_VERIFICATION.json`.

## Deployment record

Cloud Build: `b949f5ad-1f9a-46c1-ada2-c516590aa1ce`, SUCCESS,
finished `2026-09-24T20:29:31.975360Z`.

Cloud Run: `omnilink-00234-7ff`, 100% traffic after deployment.
Prior revision: `omnilink-00233-mlq`.

Published image digest:
`sha256:7541efc73fbdc0f9729bfa248a438259dc7b34f9e0506176ef24ad43fb537a78`.

The image is a static-asset overlay on the verified prior production image
`sha256:5de7bfd2832e138ead814e2174bfc6d96e42b7b7a9505ca69fc393e6060e4f77`.
Only `/app/dist/index.html`, `/app/dist/sitemap.xml` and the benchmark directory
were overlaid. Backend binaries, other application assets and service settings
were preserved. This avoided shipping unrelated source changes while GitHub
deployment jobs were failing before their steps began.

Website source remains in `O:/omnilink/index.html`, `src/seo.ts` and
`public/benchmarks/robotics-control/`. The renderer is `prepare_website.py` here.
Publication deployed directly through Cloud Build before the follow-up Git
commits. The production deployment is recorded by Cloud Build and the
verification JSON above; committing the source does not redeploy it.

## Reproduction boundary

Offline evidence verification is publicly available and passed after download.
The exact matching simulator/runtime/source release has not been established
as publicly available. The page explicitly requires it for a fresh live run;
it does not claim the ZIP contains the engine or that offline regrading is an
independent rerun. Publishing the matching OmniSim release is separate work.

## Naming correction — 2026-09-25

OmniLink is the evaluated agent-control integration. OmniSim is the simulator
used for every configuration. The homepage, chart/table labels, explanations,
page title, search/share metadata, result JSON display labels, website renderer
and next-step document were corrected to preserve that distinction. Genuine
simulator navigation and runtime/reproduction references remain OmniSim.

The result JSON was compared against the previously published version: only
`display_labels` changed. Raw evidence, numerical results, scopes, limitations,
download archives and their checksums are unchanged. The production frontend
build and 45 SEO checks passed. This is a wording correction, not a new campaign.

The correction deployed successfully through Cloud Build
`92551636-fc64-4f05-9379-db9bf5a38168`, finishing at
`2026-09-24T21:14:48.515172Z` (September 25 in Africa/Cairo). Revision
`omnilink-00235-p5c` receives 100% of traffic. Only the homepage, benchmark HTML
and result JSON were overlaid onto the prior production image. All three live
files matched their prepared hashes; archive checksums remain unchanged.
Browser verification confirmed the title, introductory wording, all six
configuration labels and the explicit simulator distinction. See
`NAMING_CORRECTION_VERIFICATION.json`.
