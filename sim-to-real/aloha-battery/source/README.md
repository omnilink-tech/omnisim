# Real ALOHA battery episode

Source: [lerobot/aloha_static_battery](https://huggingface.co/datasets/lerobot/aloha_static_battery),
revision `06dc3da83c4fd3d1889b00f1dfd3780da8421f64`. Episode 0 contains 600
frames at 50 Hz, lasting 12 seconds. The task is “Place the battery into the
slot of the remote controller.”

- `episode-000-high.mp4` and `episode-000-low.mp4`: the first 12 seconds of
  the published high and low camera streams, copied without video re-encoding.
- `episode-000.parquet`: all 600 rows of episode 0, selected from the source
  parquet. `episodes.parquet`, `tasks.parquet` and `dataset-info.json` retain
  the original metadata.
- `downloads.json`: original URLs, pinned revisions, byte sizes and SHA-256
  hashes. Its paths describe the initial download locations; full source
  camera streams and all 49 episodes are not duplicated in this folder.
- `LICENSE-2.0.txt`: the license declared by the dataset. Recording credit:
  ALOHA, Tony Zhao and collaborators; dataset distribution by LeRobot.

These are real robot recordings, not newly generated or simulated footage.
The recording does not establish that the new OmniSim controller ran on that
hardware. The original [ALOHA project page](https://tonyzhaozh.github.io/aloha/)
also hosts a [battery-task preview](https://tonyzhaozh.github.io/aloha/resources/slot_battery.mp4).
The comparison uses the dataset episode above so the real clip and joint data
come from the same recorded episode.

Local converted robot and scene assumptions are documented in the
[robot provenance](../../../projects/robots/trossen/aloha/PROVENANCE.md).
