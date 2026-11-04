# Delivery acceptance checklist

Record the purchaser/contact and delivery date in your order, not in the frozen
evidence. SKU {{SKU}} version {{VERSION}} has a US$100 standard scope.

1. Check the archive's SHA256 against the checksum supplied with delivery.
2. Extract on a Windows x64 machine. Read SCOPE.md, especially the reference
   controller and simulation-only limitations. Open REPORT.html offline.
3. Run `assessment.cmd verify`: expect verified=true and 64 regraded episodes.
4. Run `assessment.cmd accept --out runs/acceptance-01`: expect four completed
   episodes and verified=true in acceptance.json. The known failing cases must
   still FAIL, the passing ones must PASS, and forward-excursion differences
   must each be at most 0.005 m. Compare with the saved source measurements.
5. Open a new case report and inspect its trace, spec, verdict, adoption.json,
   compiled-model.txt and finalized Newton sidecar. Confirm the new run has its
   own machine fingerprint and provenance; original evidence is unchanged.
6. Record the acceptance result, machine, package checksum and any discrepancy.
   Send a consolidated clarification through the agreed delivery contact within
   14 calendar days if needed. Retain failed attempts for investigation.

For release QA, execute the same checklist from a fresh extraction, with the
developer checkout absent from Python paths and no Git command on PATH. Verify
that the included embedded Python and runtime suffice. Repeat on a separate clean
Windows machine before describing the release as verified on independent PCs.

This checklist accepts the assessment delivery. It does not accept a real robot
for deployment, validate manufacturer firmware, or certify hardware safety.
