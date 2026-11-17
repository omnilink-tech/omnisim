# Axis's knowledge folder

Drop files in this folder. Axis's `search_knowledge(query)` tool will index them on demand and return the matching passages.

This is the **curated** grounding surface — authoritative facts about robots, the OmniSim bridge, and the OmniLink platform. It's analogous to the `tools/` folder: source-controlled, auto-discovered, portable with the agent.

## What goes here

- Robot capability records (joint limits, home poses, IK constants, TCP offsets)
- OmniSim bridge schemas and endpoint contracts
- OmniLink architecture notes relevant to Axis
- Deployment topology (which bridge hosts which robot, which scenes are active)
- Axis self-reference — what tier the agent is in, which tools it holds, what authority it has

## Supported file types

- `.md` / `.markdown` — recommended
- `.txt` / `.rst`
- `.json` — values and nested keys are searched (useful for capability records)
- `.csv` — treated as text
- `.pdf` — requires `pypdf` (`pip install pypdf`); text extraction only

## What doesn't belong here

- Secrets, bridge API tokens, operator credentials. This folder is source-controlled.
- Per-tick telemetry logs (those live in durable logs, not the grounding surface).
- Binaries — URDF files, STL meshes, recorded trajectories; link to them from a markdown note instead.

## How it's retrieved

Axis calls `search_knowledge(query="...")`. The tool scans every supported file in this folder, scores passages by keyword + proximity, and returns the top matches with file, line range, and text.

## Convention

Name files by topic: `robots.md`, `omnisim-bridge.md`, `omnilink-architecture.md`, `axis-self.md`. Use headings inside — the tool chunks on headings where present.
