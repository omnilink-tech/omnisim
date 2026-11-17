# results_published/ — tracked campaign rows

**The rule: no row, no result** ([agent-edge-validation-plan §0.2](../../../../docs/developer/agent-edge-validation-plan.md)).
A number that exists only in a commit message, an operator's note, or an
agent's summary may not be quoted anywhere. Campaign rows must live in a
tracked path, or the campaign did not happen.

This directory is that path. The sibling `results/` tree is **gitignored
scratch** — every run writes there, and its rows die with the machine that
produced them. Rows land *here* only through a deliberate act:

```bash
python tests/benchmarks/agentbench/run_agentbench.py --publish results/<run_id>
```

which copies `results/<run_id>/rows.jsonl` to
`results_published/<run_id>/rows.jsonl` plus a `publish_meta.json`
(source path, row count, `rows_sha256`, publish time). The copy is then
**reviewed and committed** like any other change. Nothing auto-publishes,
and `--publish` refuses to overwrite an existing publication — replacing
reviewed rows is an explicit delete-then-publish, visible in the diff.

What a publication must survive (SPEC §3.1/§5): every row carries its own
`condition`, `machine` fingerprint, `sim_version` and metrics, so a
published row is quotable without the machine that produced it. Rows with
`n = 1` remain `exploratory` and barred from published claims (SPEC §3.5);
publishing them here preserves them, it does not license quoting them.
