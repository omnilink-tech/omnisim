# ur3e — greet

Acknowledge an operator at the cell.

⚠️ **Generated** by `scripts/dev/gen_agent_catalogue.py`. Edit the matrix there, not this directory.

A catalogue brief, not a hand-built agent: it is a robot, a task and an ordered list of things an operator would say. It does not plan, recover or reason — see `agents/production/husky_maze/` for one that does.

| | |
|---|---|
| surface | `arm` |
| world | `projects/samples/demos/worlds/chat/omnilink_ur3e.omniworld` |
| bridge | `8765` |
| steps | 3, each checked |

```bash
python scripts/dev/run_agent_catalogue.py --only ur3e_greet
```
