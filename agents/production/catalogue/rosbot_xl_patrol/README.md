# rosbot_xl — patrol

Walk a rectangular beat and halt on the operator's word.

⚠️ **Generated** by `scripts/dev/gen_agent_catalogue.py`. Edit the matrix there, not this directory.

A catalogue brief, not a hand-built agent: it is a robot, a task and an ordered list of things an operator would say. It does not plan, recover or reason — see `agents/production/husky_maze/` for one that does.

| | |
|---|---|
| surface | `mobile` |
| world | `projects/samples/demos/worlds/chat/omnilink_rosbot_xl.omniworld` |
| bridge | `8765` |
| steps | 6, each checked |

```bash
python scripts/dev/run_agent_catalogue.py --only rosbot_xl_patrol
```
