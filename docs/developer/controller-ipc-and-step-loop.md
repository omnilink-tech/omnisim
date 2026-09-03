# Controller IPC And Step Loop

This document explains how controller communication currently interacts with the simulation step loop, where the expensive or fragile edges are, and what should change in a later cleanup.

It is grounded in:

- `src/controller/c/robot.c`
- `src/omnisim/control/OmControlledWorld.cpp`
- `src/omnisim/control/OmController.cpp`

## Why This Area Matters

The controller boundary is one of the most important architectural seams in the simulator.

It affects:

- simulation throughput
- determinism
- startup behavior
- debugging quality
- coding-agent ability to validate runtime changes without the full desktop shell

Today that boundary works, but it is still too entangled with the main step loop.

## Current Flow

At a high level the controller path looks like this:

1. a robot controller process or extern controller connects through the existing socket or pipe mechanism
2. `OmController` sends step and immediate packets, flushing the socket after each write
3. `OmController::readRequest()` reconstructs controller-protocol packets from a possibly fragmented byte stream
4. `OmControlledWorld::step()` coordinates controller startup, waiting, retries, and when the main simulation step may execute
5. `OmSimulationWorld::step()` only runs once controller-side conditions are satisfied

This means the controller protocol is not just I/O. It is part of the runtime scheduler.

## Code-Backed Pressure Points

### 1. The libController stdout/stderr redirect path is bounded by a fixed buffer

In `src/controller/c/robot.c`:

- `stream_pipe_create()` uses a pipe buffer size of `65536` (64 KB) on Windows
- `stream_pipe_read()` reads into a 64 KB buffer on Windows; on POSIX it loops with a dynamic buffer capped at 4 MB
- both `wb_robot_step_begin()` and `wb_robot_step()` flush and read these redirected streams before continuing

Implications:

- noisy controllers can distort step cost and debugging behavior
- the protocol boundary has one more hidden fixed-size limit that is not obvious to contributors

Note: the Windows pipe-read cap was raised from 1024 to 64 KB (landed). Unifying it with the POSIX dynamic-growth path is still future work (see Section 3 below). Truncation of output above ~64 KB per step on Windows therefore remains possible.

### 2. First-step and retry semantics are implicit and spread across multiple code paths

`OmControlledWorld` currently handles:

- delayed controller creation on first step
- waiting controllers
- newly attached controllers during execution
- retrying blocked steps later
- a `stepBlocked` signal when the loop cannot proceed yet

This is serviceable, but it means controller lifecycle state is inferred from control flow rather than from one explicit state machine.

Symptoms in the code:

- `mFirstStep` special handling
- `retryStepLater()`
- `mRetryEnabled`
- `mHasWaitingStep`
- multiple controller buckets such as waiting, new, and disconnected extern controllers

That makes the scheduler harder to reason about than it needs to be.

### 3. Packet transport and scheduling concerns are mixed

`OmController` has to know about:

- local and TCP socket handling
- flushing behavior after writes
- initial `wb_robot_init()` packet quirks
- immediate messages versus step messages
- template-regeneration blocking side effects
- performance logging of controller time

That is too much responsibility for one class at a hot architectural seam.

### 4. Controller timing exists, but controller pressure is still under-instrumented

`OmController` already starts and stops `OmPerformanceLog::CONTROLLER` measurements per controller name.

That is useful, but it still does not tell us enough:

- packet sizes are not surfaced
- retry frequency is not surfaced
- time spent blocked waiting for controllers is not broken out cleanly
- stdout/stderr redirect overhead is not measured

The result is that contributors can see that controllers are expensive without being able to tell why.

### 5. Sensor updates and controller cadence are related but not identical

`OmControlledWorld` contains logic that updates sensors regularly even when controller scheduling differs.

That is the right general idea, but it further confirms that the controller loop and the simulation loop are coupled by policy rather than by a narrow runtime contract.

## Risks In The Current Design

The main risks are not "the code crashes every day." The risks are slower and more structural.

### Runtime risks

- waiting or retry behavior can stretch step latency in hard-to-predict ways
- controller noise can affect perceived simulation smoothness
- controller attach/detach behavior is harder to test in isolation than it should be

### Build and contributor risks

- controller changes often require understanding large portions of the runtime loop
- the right validation path is not obvious from the code structure alone
- coding agents are likely to over-edit because protocol, process, and scheduling concerns are interleaved

### Architecture risks

- there is no small controller-runtime interface yet
- controller lifecycle still leaks into world-step orchestration
- headless runtime work cannot fully ignore desktop-era assumptions

## Recommended Direction

### 1. Make controller lifecycle explicit

Introduce a documented controller state model with states such as:

- created
- starting
- waiting for init packet
- ready
- waiting for step response
- disconnected
- terminated

That replaces control-flow inference with explicit runtime state.

### 2. Separate transport, protocol, and scheduling

The current controller code should be split conceptually into:

- transport: sockets, pipes, buffering, reconnect rules
- protocol: packet framing, request and response semantics, immediate versus step messages
- scheduling: when the runtime is allowed to advance

These can still live in the same binary at first, but they should stop living as one inseparable responsibility.

### 3. Unify redirected stream reads across platforms

The Windows path in `robot.c` reads into a fixed 64 KB buffer; the POSIX path already drains dynamically up to a 4 MB hard cap. Windows should match POSIX behavior:

- dynamically sized reads that grow until the pipe is drained
- bounded but explicit safety caps for pathological output

This is both a correctness cleanup and a debugging improvement.

### 4. Measure controller pressure directly

Add metrics for:

- controller wait time before a simulation step can proceed
- packet bytes sent and received
- count of retries per world step
- stdout/stderr redirect bytes captured
- controller startup time versus steady-state controller step time

Without those numbers, controller optimization remains guesswork.

### 5. Define a headless validation contract for controller work

A controller-facing change should be testable through:

- one controller-libs build target
- one small runtime build target
- one headless controller scenario

If controller work still requires the full desktop simulator by default, the boundary is not clean enough yet.

## Low-Risk Changes To Do First

These are good early wins:

- log retry counts and controller wait time
- document controller state transitions in code comments and docs
- add one benchmark or smoke scenario that stresses controller chatter without heavy rendering

These improve measurement and debuggability before deeper refactors start.

## Phase-Two Refactor Sequence

1. add telemetry for wait time, retries, and packet pressure
2. introduce an explicit controller lifecycle model
3. isolate packet framing from world-step scheduling
4. move scheduler policy into a narrower runtime-facing coordinator

That order reduces risk because each step improves visibility before behavior changes.

## Validation Guidance

After controller-related changes, validate with:

- `python -m omnisim build controller-libs`
- `python -m omnisim build core`
- `python -m omnisim run-headless <small-world>`
- one smoke or benchmark scenario that exercises controller traffic without large render cost

The important thing is to prove that controller work can be validated narrowly and repeatably.
