# Benchmark publication handoff

Campaign publication-04 is complete and verified: 198/198 episodes, six
implementations, 33 tasks each, one repeat. All implementations passed 32/33
and recorded one unwanted-motion failure on the same vague arm instruction.
OmniLink used 42 model requests versus 73 for the plain loop; estimated model
cost per success was $0.012440 versus $0.020942 (40.6% lower in this run).
Parser-enabled framework controls were close to OmniLink. No market-wide
leadership or superior task-completion claim is supported.

Read ../PUBLICATION_REPORT.md for the complete table, uncertainty, shared arm
limitation and the exact interpretation. Earlier incomplete campaigns remain
preserved, and the owner authorized the diagnostics. No approval is pending.

## Prepared website

Local website changes:
- O:/omnilink/index.html: measured benchmark section after the existing demo cost.
- O:/omnilink/public/landing.css: scoped spacing and mobile card layout.
- O:/omnilink/public/benchmarks/robotics-control/: full page and evidence downloads.

The original $0.73 six-instruction demo remains separate. The new section names
its task denominator, developer-authored provenance, unwanted-motion count,
model-cost basis and scope. Every comparison arm and failure is visible.
Nothing has been committed or deployed by this benchmark task.

Local preview: http://127.0.0.1:8879/#robotics-benchmark
Full table: http://127.0.0.1:8879/benchmarks/robotics-control/index.html
Generated artifacts: publication-preview-04/

## Verification

40 benchmark tests passed. Fresh archive extraction passed Python -I -S with
1,145 file hashes and all 198 scores/costs/model identities/physics proofs matching.
All 347 model requests returned usage and standard service tier; no error-cost
assumption was used. All 50 Python dependency versions matched their lock.
The provider/account profile was unchanged during/after the run.

The production website build passed; desktop and mobile browser checks passed.
All seven local download/document links exist. Both archive checksums match in
the generated artifacts, website public source and built website output.

Evidence ZIP SHA256:
62d2d8afa7b3e4145ff57f36949906f863ebe79438e40b2187c2dd1c95dc9892

## Release dependency

Publish with the matching OmniSim 9.0.0 source and Windows runtime, plus the
archived measured source overlay. Verify a clean released installation before
advertising public live reproduction. Offline score verification already works.
A later arm-gate fix must not silently change these historical measurements.

The source-release deny-list now excludes raw robotics benchmark working
evidence, locks and diagnostic JSON. Public evidence is exported separately,
with privacy handling and hashes. Do not pass immutable archives through a text
redaction step or substitute redacted raw rows without the disclosed manifest.

After deployment, download the evidence asset from its public URL, compare this
SHA256 and run verify_archive.py in a fresh directory. This task has not deployed
or independently replicated the live study on another machine/account.
