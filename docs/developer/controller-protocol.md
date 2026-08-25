# OmniSim Controller Protocol and Lifecycle

This document describes the controller lifecycle state machine, the request/answer protocol, and the synchronization model between the simulator and robot controllers. It is aimed at contributors who need to debug controller behavior or modify the controller subsystem.

## Overview

Each robot in a world can have a controller — an external process (C, C++, Python) that sends commands to the robot's actuators and reads its sensors. The simulator and controllers communicate through a binary IPC protocol over local Unix sockets or TCP connections.

## Controller Lifecycle States

```
                ┌─────────────┐
                │   CREATED   │  OmController constructed, not yet started
                └──────┬──────┘
                       │ start()
                       ▼
         ┌─────────────────────────────┐
         │                             │
    ┌────▼────┐                 ┌──────▼──────┐
    │ WAITING │ extern=true     │  LAUNCHING  │ extern=false
    │ (extern)│                 │  (process)  │
    └────┬────┘                 └──────┬──────┘
         │ addLocal/Remote             │ QProcess::started
         │ Connection()                │ addLocalControllerConnection()
         ▼                             ▼
    ┌─────────────────────────────────────┐
    │             CONNECTED               │  Socket established, initial
    │                                     │  configure handshake done
    └──────────────┬──────────────────────┘
                   │
                   │ normal operation (request/answer loop)
                   ▼
    ┌─────────────────────────────────────┐
    │              RUNNING                │  Controller sends step requests,
    │                                     │  simulator sends answers
    └────────┬────────────┬───────────────┘
             │            │
    disconnect()    processFinished()
             │            │
             ▼            ▼
    ┌────────────┐  ┌─────────────┐
    │DISCONNECTED│  │ TERMINATED  │
    │  (extern)  │  │ (process)   │
    └────────┬───┘  └──────┬──────┘
             │             │
             │ (extern:    │ hasTerminatedByItself signal
             │  waits for  │ → deleteController()
             │  reconnect) │
             ▼             ▼
    ┌────────────┐  ┌─────────────┐
    │  WAITING   │  │   DELETED   │
    │  (again)   │  │             │
    └────────────┘  └─────────────┘
```

## Key Classes

### OmController (`src/omnisim/control/OmController.*`)
Owns one controller's lifecycle:
- Process management (QProcess for internal controllers)
- Socket management (QLocalSocket for local IPC, QTcpSocket for remote)
- Request parsing and answer writing
- stdout/stderr forwarding to the simulator console

### OmControlledWorld (`src/omnisim/control/OmControlledWorld.*`)
Orchestrates all controllers and synchronizes them with the simulation step:
- Maintains controller lists: `mControllers`, `mWaitingControllers`, `mNewControllers`, `mTerminatingControllers`, `mDisconnectedExternControllers`
- Implements the step synchronization protocol
- Manages the retry-step-later mechanism for synchronized controllers

## Controller Lists in OmControlledWorld

| List | Purpose |
|------|---------|
| `mControllers` | Currently running controllers (both internal and extern connected) |
| `mWaitingControllers` | Controllers added in previous step, will be started in current step |
| `mNewControllers` | Controllers added in current step, waiting for next step |
| `mTerminatingControllers` | Controllers waiting to be deleted |
| `mDisconnectedExternControllers` | Extern controllers that started but have no connection yet |

A controller should be in exactly one list at any time.

## Request/Answer Protocol

### Packet Format
All packets use little-endian byte order.

**Request (controller → simulator):**
```
[4 bytes] packet_size (uint32, includes this field)
[4 bytes] step_duration_ms (uint32)
[N bytes] device messages (parsed by robot->dispatchMessage)
```

- `step_duration_ms = 0` with `isConfigureMessage=true`: initial configure handshake
- `step_duration_ms = 0` with `isConfigureMessage=false`: immediate message (supervisor API call)
- `step_duration_ms > 0`: normal step request — controller wants to advance by N ms

**Answer (simulator → controller):**
```
[4 bytes] packet_size (uint32, TCP only)
[4 bytes] delay (int32, simulation time elapsed)
[N bytes] device answers (sensor data, assembled by robot->dispatchAnswer)
```

### Connection Handshake
1. Controller process starts and connects to the local socket
2. `addLocalControllerConnection()` is called
3. Controller sends `wb_robot_init` packet (robotId + step(0) configure request)
4. Simulator reads the configure request (`readRequest()`)
5. Simulator sends the configure answer (`writeAnswer()`)
6. Normal request/answer loop begins

### Normal Step Flow
1. Controller calls `wb_robot_step(ms)` → sends request packet with `step_duration_ms = ms`
2. `readRequest()` parses the packet, dispatches device messages, records `mDeltaTimeRequested`
3. Simulation advances by the world's `basicTimeStep` via `OmControlledWorld::step()`
4. When the accumulated simulation time >= controller's requested time, `writeAnswer()` sends sensor data back
5. Controller receives sensor data and continues execution

### Immediate Messages (Supervisor)
When a supervisor controller needs to query or modify the world between steps:
- It sends a request with `step_duration_ms = 0` (and `isConfigureMessage = false`)
- The simulator processes it immediately and sends an answer
- This does not advance simulation time

## Synchronization Model

### Synchronized Controllers (`synchronization = TRUE`, default)
- The simulator waits for all synchronized controllers to send their step requests before advancing
- If a controller hasn't sent its request yet, `needToWait()` returns true and `retryStepLater()` is called
- The step is retried when the controller's `requestReceived` signal fires
- This ensures deterministic simulation

### Asynchronous Controllers (`synchronization = FALSE`)
- The simulator does not wait for the controller
- The controller runs at its own pace
- Sensor data may be stale

### Extern Controllers
- `mExtern = true`: no QProcess is created
- The simulator listens on a local socket and optionally a TCP port
- The extern controller connects from outside (another process, another machine)
- On disconnect, the simulator goes back to WAITING state (does not terminate the controller slot)
- On reconnect, a new connection is accepted and the handshake repeats

## Step Execution Flow in OmControlledWorld::step()

```
1. Start any waiting controllers (mWaitingControllers → mControllers)
2. Check if we need to wait for controllers (needToWait)
   - If waiting for extern controller connection → pause step timer
   - If waiting for controller request → retryStepLater, return
3. Send answers to controllers whose requested time has elapsed
4. Call OmSimulationWorld::step() to advance physics
5. Process any newly added controllers (mNewControllers → mWaitingControllers)
6. Process any terminating controllers
7. Request render update if needed
```

## Transport Paths

### Local IPC (QLocalSocket)
- Socket path: `<tmp>/ipc/<robot_encoded_name>/intern` or `extern`, where `<tmp>` is the
  per-instance temporary folder (set via `OMNISIM_TMPDIR`; `WEBOTS_TMPDIR` is still read as a
  legacy alias — [`OmStandardPaths.cpp:277-280`](../../src/omnisim/core/OmStandardPaths.cpp#L277))
- On Windows: named pipe
  - **intern** controller: `webots-<tmpId>-<simulatorPid>-<robot_encoded_name>`
  - **extern** controller: `webots-<tmpId>-<robot_encoded_name>`

> ⛔ **The `webots-` pipe prefix above is REAL and must not be "rebranded".** It is the literal
> wire name both sides build — see [`robot.c:1315`](../../src/controller/c/robot.c#L1315) and
> `:1319` (`snprintf(socket_filename, length, "\\\\.\\pipe\\webots-%d-%s-%s", ...)`). Renaming it
> in this document alone would make the document false; renaming it in code is an engine↔
> libController ABI break of exactly the kind that silently hangs every controller at zero ticks.
- Used for both internal controllers and local extern controllers
- The `<simulatorPid>` nonce on the intern name (exported to the child as
  `OMNISIM_IPC_NONCE`, appended on both sides — `OmController::start` and libController's
  `compute_socket_filename` in `robot.c`) makes the pipe name unique per launch. Without it,
  back-to-back headless launches that reuse the same TCP port (`<tmpId>`) built an identical
  pipe name; since Windows allows multiple server instances of a name, a fresh child could
  `CreateFile(OPEN_EXISTING)` onto the previous launch's lingering instance and cross the
  pairing (the residual launch-flake race, `default-flip-plan.md` §3.5). Extern controllers
  are user-launched without the nonce, so their name stays unsalted and libController's extern
  autodetect reconstructs it unchanged.

### Remote TCP (QTcpSocket)
- Connected via `OmTcpServer` on the configured port (default: 1234)
- Packet format adds a 4-byte size prefix for TCP framing
- Used for remote extern controllers

## Debugging Tips

- Controller stdout/stderr is buffered in `mStdoutBuffer`/`mStderrBuffer` and flushed at step boundaries for deterministic log ordering
- Enable `--stdout --stderr` flags to see controller output in the terminal
- The `omnisim_log.txt` file captures all INFO/WARNING/ERROR messages including controller start/stop events
- `mIncompleteRequest` flag indicates a partially received TCP packet (fragmentation)
- `mProcessingRequest` flag prevents re-entrant request handling

## Files

| File | Role |
|------|------|
| `src/omnisim/control/OmController.*` | Single controller lifecycle and protocol |
| `src/omnisim/control/OmControlledWorld.*` | Multi-controller orchestration and step sync |
| `src/controller/c/robot.c` | C runtime: `wb_robot_init`, `wb_robot_step`, request/answer impl |
| `src/controller/c/request.c` | Request packet assembly |
| `src/controller/c/scheduler.c` | Step scheduling on the controller side |
| `src/controller/launcher/omnisim_controller.c` | Standalone extern controller launcher |
| `src/omnisim/gui/OmTcpServer.*` | TCP server for remote extern controllers |
