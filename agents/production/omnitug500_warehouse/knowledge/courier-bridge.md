# OmniTug 500 courier bridge — contract

The agent drives the `omnitug500_courier` controller over HTTP (default
`127.0.0.1:8765`, brought up by `omnitug500_courier.omniworld`). All calls are POST with a
JSON body except the two reads, which also accept GET.

| Tool | Endpoint | Body | Returns |
|---|---|---|---|
| `list_stations` | `/capabilities` | — | `{model, deck_capacity, pickup_bays:[{name,label,color}], docks:[...], packages:[{name,staged_at}]}` |
| `get_courier_state` | `/get_robot_state` | — | `{x, y, yaw_deg, speed, mode, active, queue, carrying:[...], deck_free, at_station, last_event, fault, sim_time}` |
| `goto_station` | `/goto_station` | `{station}` | `{accepted, op, station, eta_s}` |
| `pick_package` | `/pick_package` | `{station?, package?}` | `{accepted, op:"pick", station, package, eta_s}` |
| `deliver_package` | `/deliver_package` | `{station, package?}` | `{accepted, op:"deliver", ...}` |
| `run_route` | `/run_route` | `{steps:[{action,station,package?}]}` | `{accepted, steps, route:[...]}` |
| `stop_rover` | `/stop` | — | `{accepted, halted_at}` |
| `reset_demo` | `/reset` | — | `{accepted}` |

## Asynchronous execution

`goto_station` / `pick_package` / `deliver_package` / `run_route` **enqueue** work
and return immediately. The rover executes the queue one step at a time in its
control loop. To know when a step (or the whole route) has finished, poll
`get_courier_state`:

- `mode`: `idle` (nothing to do) | `drive` (routing) | `align` (final heading
  turn at the anchor) | `act` (loading/unloading).
- `queue`: remaining steps. The run is complete when `mode == "idle"` **and**
  `queue == 0`.
- `carrying`: the packages currently on the deck. It grows on a pick and empties
  on a deliver.
- `last_event`: a human line — `"loaded pkg_a (1/3 on deck)"`,
  `"delivered pkg_a, pkg_c to Dock 3"`, `"no route to ..."`, etc.
- `fault`: non-null only on an error (e.g. a step watchdog timeout). Report it.

## Errors

A bad station/package name returns `{"error": ...}` with a `known`/`bays`/`docks`
list — re-read `list_stations` and retry with a valid name. `pick_package`
fails if the deck is already full (`deck_free == 0`).
