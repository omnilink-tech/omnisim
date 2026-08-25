# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""cuda_particle_pool — Phase 1.5 prototype of the CUDA particle field.

Owns a pre-allocated pool of N sprite Solids in the scene. Particles
live in a numba.cuda buffer (state: x,y,z,vx,vy,vz,life,size). Each
sim step we run an advance kernel on the GPU (gravity + drag + life
decay + floor bounce), copy positions back, and `setSFVec3f` each
Solid's translation. Out-of-life slots get scaled to 0 (effectively
hidden) until they're recycled by the next spawn.

Talks to other controllers / external clients over a small TCP wire
protocol on port 6791:
  spawn(pos, vel, life, size, color)  -> particle index
  state()                              -> dict of pool stats
  ping()                               -> {}

Designed to replace the per-particle importMFNodeFromString path used
by Phase 11 smoke / spark spawning in damage_tracker. The C++ version
of this (P2-P7 in cuda-particle-effects-plan.md) will be much faster
because it owns the GL/CUDA interop directly; this prototype proves
the architecture works without requiring a Webots binary rebuild.

See docs/developer/cuda-particle-effects-plan.md.
"""

from __future__ import annotations

import json
import os
import select
import socket
import struct
import sys
import time
from typing import Optional

import numpy as np

try:
    from numba import cuda
    HAVE_CUDA = cuda.is_available()
except ImportError:
    HAVE_CUDA = False
    cuda = None  # type: ignore

from omnisim import Supervisor


HOST = os.environ.get("OMNISIM_PARTICLE_POOL_HOST", "127.0.0.1")
PORT = int(os.environ.get("OMNISIM_PARTICLE_POOL_PORT", "6791"))
POOL_SIZE = int(os.environ.get("OMNISIM_PARTICLE_POOL_SIZE", "200"))
SOLID_DEF_PREFIX = "particle_"

# Particle state layout: 8 fields per particle in a (N, 8) float32 array.
# Field indices:
F_X, F_Y, F_Z = 0, 1, 2
F_VX, F_VY, F_VZ = 3, 4, 5
F_LIFE = 6
F_SIZE = 7

GRAVITY = -9.81
DRAG = 0.4


if HAVE_CUDA:
    @cuda.jit
    def advance_particles_kernel(particles, dt, gravity, drag):
        i = cuda.grid(1)
        n = particles.shape[0]
        if i >= n:
            return
        # Skip dead particles (life <= 0). The CPU reads them as far-away
        # to hide the Solid; we still run the kernel on every slot for
        # SIMD efficiency — branching here would just add a divergent path.
        life = particles[i, F_LIFE]
        if life <= 0.0:
            return
        # Apply gravity
        particles[i, F_VZ] += gravity * dt
        # Linear drag on all velocity components
        decay = 1.0 - drag * dt
        particles[i, F_VX] *= decay
        particles[i, F_VY] *= decay
        particles[i, F_VZ] *= decay
        # Integrate position
        particles[i, F_X] += particles[i, F_VX] * dt
        particles[i, F_Y] += particles[i, F_VY] * dt
        particles[i, F_Z] += particles[i, F_VZ] * dt
        # Floor clamp + bounce restitution
        if particles[i, F_Z] < 0.02:
            particles[i, F_Z] = 0.02
            particles[i, F_VZ] = -particles[i, F_VZ] * 0.3
            # Lateral friction loss when hitting ground
            particles[i, F_VX] *= 0.7
            particles[i, F_VY] *= 0.7
        # Decrement life
        particles[i, F_LIFE] = life - dt


class ParticlePool:
    """Wraps a numba.cuda buffer of N particles plus the corresponding
    Webots Solids that visualize them. Caches the Field handles so
    per-step updates don't re-walk the scene tree.
    """

    def __init__(self, supervisor: Supervisor, n: int):
        self.supervisor = supervisor
        self.n = n
        # Host-side mirror of particle state. We keep it in pinned-ish
        # memory by way of a numpy array; numba.cuda copies it to/from
        # device each step.
        self.host = np.zeros((n, 8), dtype=np.float32)
        # Mark all initially dead (life=0)
        self.host[:, F_LIFE] = 0.0
        self.host[:, F_SIZE] = 0.05
        self.host[:, F_X] = 0.0
        self.host[:, F_Y] = 0.0
        self.host[:, F_Z] = -100.0  # park dead particles below floor

        if HAVE_CUDA:
            # Allocate the device buffer ONCE; reuse it across steps via
            # in-place copy_to_device / copy_to_host. Re-allocating with
            # cuda.to_device() every step adds ~0.5-1 ms overhead per
            # step (cudaMalloc + cudaFree), which dwarfs the actual
            # kernel cost at N<=1k.
            self.device = cuda.device_array_like(self.host)
            self.device.copy_to_device(self.host)
            self.use_cuda = True
        else:
            self.device = None
            self.use_cuda = False

        # Resolve the Solid handles + cache translation field pointers.
        # Note: Webots' Solid node has no `scale` field (that's a
        # Transform/Pose thing) so we can't shrink dead particles per-
        # slot. Instead we park them at z=-100 (off-screen). Fixed-radius
        # geometry. Variable-size particles come with the C++ rebuild.
        self.solids: list = []
        self.translation_fields: list = []
        for i in range(n):
            def_name = f"{SOLID_DEF_PREFIX}{i:04d}"
            node = supervisor.getFromDef(def_name)
            if node is None:
                sys.stderr.write(
                    f"[cuda_particle_pool] missing DEF {def_name!r} in scene; "
                    f"got {len(self.solids)} of {n} solids before this gap\n"
                )
                break
            tf = node.getField("translation")
            if tf is None:
                sys.stderr.write(
                    f"[cuda_particle_pool] DEF {def_name!r} has no translation field; "
                    f"skipping\n"
                )
                continue
            self.solids.append(node)
            self.translation_fields.append(tf)

        self.next_slot = 0
        self.spawn_count = 0
        self.kernel_ms_acc = 0.0
        self.kernel_ms_n = 0
        # Live-particle counter for telemetry
        self.live_count = 0

        sys.stderr.write(
            f"[cuda_particle_pool] pool initialized: n={n} "
            f"solids_resolved={len(self.solids)} cuda={self.use_cuda}\n"
        )
        sys.stderr.flush()

    def spawn(self, pos, vel, life: float = 1.0, size: float = 0.05) -> int:
        """Pick the next free slot (round-robin), set its state. Returns
        the slot index. Overwrites the oldest particle if all slots are
        in use; callers shouldn't depend on exact lifetime guarantees."""
        if not self.solids:
            return -1
        idx = self.next_slot
        self.next_slot = (self.next_slot + 1) % len(self.solids)
        self.host[idx, F_X] = pos[0]
        self.host[idx, F_Y] = pos[1]
        self.host[idx, F_Z] = pos[2]
        self.host[idx, F_VX] = vel[0]
        self.host[idx, F_VY] = vel[1]
        self.host[idx, F_VZ] = vel[2]
        self.host[idx, F_LIFE] = life
        self.host[idx, F_SIZE] = size
        # Note: spawn() touches self.host only. self.device gets
        # synced via the per-step copy_to_device() in step(). Skipping
        # the per-spawn upload avoids hundreds of micro-transfers when
        # spawn_burst dumps 50 particles at once.
        self.spawn_count += 1
        return idx

    def step(self, dt: float):
        """Advance all particles one step + render their positions/scales."""
        if not self.solids:
            return
        # Fast path: if nothing's alive, skip kernel + copies entirely.
        # Most of a sim's lifetime there are no particles flying; the
        # constant-per-step launch+sync overhead would otherwise dominate.
        # We do still need to make sure dead slots are parked off-screen,
        # but that's idempotent so we can skip the per-step rewrite once
        # they've been parked (`_parked_dead`).
        any_alive = bool((self.host[:, F_LIFE] > 0).any())
        if not any_alive:
            if not getattr(self, "_parked_dead", False):
                for i in range(len(self.solids)):
                    self.translation_fields[i].setSFVec3f([0.0, 0.0, -100.0])
                self._parked_dead = True
            self.live_count = 0
            return
        self._parked_dead = False
        if self.use_cuda:
            t0 = time.perf_counter()
            threads = 256
            blocks = (self.n + threads - 1) // threads
            # In-place upload host -> existing device buffer (no realloc)
            self.device.copy_to_device(self.host)
            advance_particles_kernel[blocks, threads](self.device, dt, GRAVITY, DRAG)
            self.device.copy_to_host(self.host)
            cuda.synchronize()
            self.kernel_ms_acc += (time.perf_counter() - t0) * 1000.0
            self.kernel_ms_n += 1
        else:
            # CPU fallback: same math in numpy
            self.host[:, F_VZ] += GRAVITY * dt
            decay = 1.0 - DRAG * dt
            self.host[:, F_VX:F_VZ + 1] *= decay
            self.host[:, F_X:F_Z + 1] += self.host[:, F_VX:F_VZ + 1] * dt
            below = self.host[:, F_Z] < 0.02
            self.host[below, F_Z] = 0.02
            self.host[below, F_VZ] = -self.host[below, F_VZ] * 0.3
            self.host[below, F_VX] *= 0.7
            self.host[below, F_VY] *= 0.7
            self.host[:, F_LIFE] -= dt

        # Update Solid translations. Dead particles get parked once at
        # z=-100 (when they transition from live → dead) and then left
        # alone — re-parking every step burns ~200 setSFVec3f calls per
        # step for nothing (the parked position never changes). Tracking
        # `_was_alive` per slot lets us hit setSFVec3f only on slots
        # that actually moved this step.
        if not hasattr(self, "_was_alive"):
            self._was_alive = [False] * len(self.solids)
        live = 0
        for i in range(len(self.solids)):
            life = self.host[i, F_LIFE]
            if life > 0:
                live += 1
                self.translation_fields[i].setSFVec3f([
                    float(self.host[i, F_X]),
                    float(self.host[i, F_Y]),
                    float(self.host[i, F_Z]),
                ])
                self._was_alive[i] = True
            elif self._was_alive[i]:
                # Just died this step; park once.
                self.translation_fields[i].setSFVec3f([0.0, 0.0, -100.0])
                self._was_alive[i] = False
            # else: slot has been parked already; leave it alone.
        self.live_count = live

    def stats(self) -> dict:
        mean_kernel_ms = (self.kernel_ms_acc / self.kernel_ms_n
                          if self.kernel_ms_n else 0.0)
        return {
            "n": self.n,
            "solids_resolved": len(self.solids),
            "live": self.live_count,
            "spawn_count_total": self.spawn_count,
            "next_slot": self.next_slot,
            "use_cuda": self.use_cuda,
            "mean_kernel_ms": round(mean_kernel_ms, 4),
            "kernel_samples": self.kernel_ms_n,
        }


# Wire-protocol server -- length-prefixed JSON, same shape as harness_supervisor

def _recv_exact(sock, n):
    chunks = []
    remaining = n
    while remaining > 0:
        try:
            chunk = sock.recv(remaining)
        except (socket.timeout, OSError):
            return None
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _read_frame(sock):
    h = _recv_exact(sock, 4)
    if h is None:
        return None
    (length,) = struct.unpack(">I", h)
    if length == 0 or length > 1024 * 1024:
        return None
    body = _recv_exact(sock, length)
    if body is None:
        return None
    try:
        return json.loads(body.decode("utf-8"))
    except json.JSONDecodeError:
        return None


def _write_frame(sock, obj):
    body = json.dumps(obj).encode("utf-8")
    try:
        sock.sendall(struct.pack(">I", len(body)) + body)
        return True
    except OSError:
        return False


def main() -> int:
    # File-based debug because controller stderr doesn't reach
    # omnisim_log.txt. Path is repo-relative regardless of cwd.
    import time as _time
    import traceback as _tb
    _dbg_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..",
                     "tmp_pool_debug.log"))
    _dbg = open(_dbg_path, "a", buffering=1)
    _dbg.write(f"\n=== cuda_particle_pool start at {_time.strftime('%H:%M:%S')} ===\n")
    _dbg.write(f"  HAVE_CUDA={HAVE_CUDA}  POOL_SIZE={POOL_SIZE}\n")
    try:
        supervisor = Supervisor()
        basic_step_ms = int(supervisor.getBasicTimeStep())
        _dbg.write(f"  Supervisor() OK; basic_step_ms={basic_step_ms}\n")
    except Exception as exc:
        _dbg.write(f"  Supervisor() FAILED: {exc}\n{_tb.format_exc()}\n")
        return 1

    try:
        pool = ParticlePool(supervisor, POOL_SIZE)
        _dbg.write(f"  ParticlePool init OK; solids={len(pool.solids)}\n")
    except Exception as exc:
        _dbg.write(f"  ParticlePool() FAILED: {exc}\n{_tb.format_exc()}\n")
        return 1
    if not pool.solids:
        sys.stderr.write("[cuda_particle_pool] no particle Solids found in scene; idling\n")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((HOST, PORT))
    except OSError as exc:
        _dbg.write(f"  bind {HOST}:{PORT} FAILED: {exc}\n")
        sys.stderr.write(f"[cuda_particle_pool] bind {HOST}:{PORT} failed: {exc}\n")
        return 1
    _dbg.write(f"  bind {HOST}:{PORT} OK\n")
    server.listen(4)
    server.setblocking(False)
    sys.stderr.write(f"[cuda_particle_pool] listening on {HOST}:{PORT}\n")
    sys.stderr.flush()

    clients: list = []
    dt_s = basic_step_ms / 1000.0

    _dbg.write("  entering main loop\n")
    step_count = 0
    try:
        while supervisor.step(basic_step_ms) != -1:
            step_count += 1
            if step_count <= 3 or step_count % 100 == 0:
                _dbg.write(f"  step {step_count} sim_time={supervisor.getTime():.2f}s\n")
            pool.step(dt_s)

        # Accept new connections. Brief recv-side timeout so a
        # half-sent frame doesn't wedge the main loop waiting on
        # the rest -- we'd rather drop a slow client than starve
        # the simulation step + other clients.
        try:
            while True:
                cli, _ = server.accept()
                cli.setblocking(True)
                cli.settimeout(0.1)
                clients.append(cli)
                _dbg.write(f"  step {step_count}: accepted client; total={len(clients)}\n")
        except BlockingIOError:
            pass

        # Drain ready clients. _read_frame returns None on socket
        # timeout (set above to 100 ms) so a slow / partial-frame
        # client gets dropped instead of wedging the loop.
        if clients:
            ready, _, _ = select.select(clients, [], [], 0)
            if ready:
                _dbg.write(f"  step {step_count}: {len(ready)} ready of {len(clients)}\n")
            for cli in ready:
                try:
                    req = _read_frame(cli)
                except (socket.timeout, OSError) as e:
                    _dbg.write(f"  drain: read_frame raised: {e}\n")
                    req = None
                if req is None:
                    _dbg.write(f"  drain: closing client (no frame)\n")
                    try:
                        cli.close()
                    except OSError:
                        pass
                    clients.remove(cli)
                    continue
                _dbg.write(f"  drain: req cmd={req.get('cmd')!r}\n")
                req_id = req.get("id", 0)
                cmd = req.get("cmd", "")
                args = req.get("args") or {}
                try:
                    if cmd == "ping":
                        result = {}
                    elif cmd == "spawn_oneway":
                        # Fire-and-forget spawn: same args as `spawn` but
                        # no reply is sent. Lets damage_tracker queue many
                        # spawns without ever blocking on a recv. The pool
                        # main loop drains them lazily when its sim step
                        # has spare time.
                        pool.spawn(
                            args.get("pos", [0.0, 0.0, 1.0]),
                            args.get("vel", [0.0, 0.0, 0.0]),
                            float(args.get("life", 1.0)),
                            float(args.get("size", 0.05)),
                        )
                        # Skip the _write_frame path; jump back to the
                        # outer loop without sending anything.
                        continue
                    elif cmd == "spawn":
                        idx = pool.spawn(
                            args.get("pos", [0.0, 0.0, 1.0]),
                            args.get("vel", [0.0, 0.0, 0.0]),
                            float(args.get("life", 1.0)),
                            float(args.get("size", 0.05)),
                        )
                        result = {"slot": idx}
                    elif cmd == "spawn_burst":
                        # args.bursts: list of dicts; spawn many in one RPC
                        slots = []
                        for b in args.get("bursts", []):
                            slots.append(pool.spawn(
                                b.get("pos", [0.0, 0.0, 1.0]),
                                b.get("vel", [0.0, 0.0, 0.0]),
                                float(b.get("life", 1.0)),
                                float(b.get("size", 0.05)),
                            ))
                        result = {"slots": slots, "count": len(slots)}
                    elif cmd == "stats":
                        result = pool.stats()
                    else:
                        _write_frame(cli, {
                            "id": req_id, "ok": False,
                            "error": f"unknown cmd: {cmd}",
                        })
                        continue
                    _write_frame(cli, {"id": req_id, "ok": True, "result": result})
                except Exception as exc:  # noqa: BLE001
                    _write_frame(cli, {
                        "id": req_id, "ok": False,
                        "error": str(exc),
                    })
    except Exception as exc:
        _dbg.write(f"  main-loop exception at step {step_count}: {exc}\n{_tb.format_exc()}\n")
        return 1
    _dbg.write(f"  main loop exited cleanly at step {step_count}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
