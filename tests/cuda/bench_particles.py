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

"""Particle-system CUDA proof-of-concept benchmark.

Demonstrates the speedup case for a CUDA-accelerated replacement of the
damage system's per-particle smoke / spark / debris spawning. Today each
particle is imported into the world via importMFNodeFromString — single-
threaded, blocking, multi-millisecond per call. The bench measures:

  GPU : numba.cuda kernel advancing N particle states (pos, vel, life)
  CPU : pure-numpy single-threaded reference of the same kernel
  WB  : OmniSim wire-protocol round-trip latency under sustained load
        (proxy for current per-spawn cost since each importMFNodeFromString
         call goes through OmniSim's single-threaded VRML parser)

Sweeps N over [100, 1k, 10k, 100k] and prints a speedup table. The
numbers serve the same purpose as bench_granular_group.py served for
the GranularGroup CUDA path: justify the C++ implementation effort
with measured speedup before committing to a multi-day binary rebuild.

Usage:
    python tests/cuda/bench_particles.py
"""
from __future__ import annotations

import json
import socket
import struct
import sys
import time

import numpy as np

try:
    from numba import cuda
    HAVE_CUDA = cuda.is_available()
except ImportError:
    HAVE_CUDA = False
    cuda = None

PARTICLE_FIELDS = 8  # x,y,z,vx,vy,vz,life,size
GRAVITY = -9.81
DRAG = 0.4
DT = 0.016


if HAVE_CUDA:
    @cuda.jit
    def advance_particles_gpu(particles, dt, gravity, drag):
        i = cuda.grid(1)
        n = particles.shape[0]
        if i >= n:
            return
        particles[i, 5] += gravity * dt
        decay = 1.0 - drag * dt
        particles[i, 3] *= decay
        particles[i, 4] *= decay
        particles[i, 5] *= decay
        particles[i, 0] += particles[i, 3] * dt
        particles[i, 1] += particles[i, 4] * dt
        particles[i, 2] += particles[i, 5] * dt
        particles[i, 6] -= dt
        # Floor clamp + bounce restitution
        if particles[i, 2] < 0.0:
            particles[i, 2] = 0.0
            particles[i, 5] = -particles[i, 5] * 0.3


def init_particles(n: int) -> np.ndarray:
    rng = np.random.default_rng(42)
    p = np.zeros((n, PARTICLE_FIELDS), dtype=np.float32)
    p[:, 0] = rng.uniform(-1, 1, n)
    p[:, 1] = rng.uniform(-1, 1, n)
    p[:, 2] = rng.uniform(0.5, 1.5, n)
    p[:, 3] = rng.uniform(-3, 3, n)
    p[:, 4] = rng.uniform(-3, 3, n)
    p[:, 5] = rng.uniform(0, 5, n)
    p[:, 6] = rng.uniform(0.5, 2.0, n)
    p[:, 7] = rng.uniform(0.05, 0.15, n)
    return p


def bench_cpu(n: int, steps: int) -> float:
    p = init_particles(n)
    t0 = time.perf_counter()
    for _ in range(steps):
        p[:, 5] += GRAVITY * DT
        p[:, 3:6] *= (1.0 - DRAG * DT)
        p[:, 0:3] += p[:, 3:6] * DT
        p[:, 6] -= DT
        below = p[:, 2] < 0.0
        p[below, 2] = 0.0
        p[below, 5] = -p[below, 5] * 0.3
    return (time.perf_counter() - t0) * 1000.0 / steps


def bench_gpu(n: int, steps: int) -> float:
    if not HAVE_CUDA:
        return float("nan")
    p_host = init_particles(n)
    d_p = cuda.to_device(p_host)
    threads = 256
    blocks = (n + threads - 1) // threads
    advance_particles_gpu[blocks, threads](d_p, DT, GRAVITY, DRAG)  # warm-up
    cuda.synchronize()
    t0 = time.perf_counter()
    for _ in range(steps):
        advance_particles_gpu[blocks, threads](d_p, DT, GRAVITY, DRAG)
    cuda.synchronize()
    return (time.perf_counter() - t0) * 1000.0 / steps


def bench_webots_rtt(samples: int = 200) -> float:
    """Time wire-protocol RTT to a running supervisor as a proxy for the
    current per-spawn cost (each importMFNodeFromString blocks the same
    main thread that handles wire-protocol commands). Returns ms / call.
    """
    s = socket.create_connection(("127.0.0.1", 6790), timeout=5)
    s.settimeout(60)

    def call(cmd):
        p = json.dumps({"id": 1, "cmd": cmd, "args": {}}).encode()
        s.sendall(struct.pack(">I", len(p)) + p)
        h = b""
        while len(h) < 4:
            h += s.recv(4 - len(h))
        nb = struct.unpack(">I", h)[0]
        b = b""
        while len(b) < nb:
            b += s.recv(nb - len(b))

    t0 = time.perf_counter()
    for _ in range(samples):
        call("ping")
    elapsed = time.perf_counter() - t0
    s.close()
    return elapsed * 1000.0 / samples


def main():
    counts = [100, 1000, 10000, 100000]
    steps = 100
    print(f"\nParticle CUDA benchmark -- GPU compute vs CPU vs OmniSim RTT")
    print(f"steps={steps}, dt={DT}s, gravity={GRAVITY}, drag={DRAG}")
    print(f"CUDA available: {HAVE_CUDA}\n")
    print(f"{'N':>8}  {'CPU ms/step':>14}  {'GPU ms/step':>14}  {'speedup':>10}")
    print("-" * 56)
    rows = []
    for n in counts:
        cpu_ms = bench_cpu(n, steps)
        gpu_ms = bench_gpu(n, steps)
        speedup = cpu_ms / gpu_ms if gpu_ms == gpu_ms and gpu_ms > 0 else float("nan")
        rows.append((n, cpu_ms, gpu_ms, speedup))
        print(f"{n:>8}  {cpu_ms:>11.3f}    {gpu_ms:>11.3f}    {speedup:>8.1f}x")

    print(f"\nReal-time budget at basicTimeStep=16 ms. GPU headroom:")
    for n, _, gpu, _ in rows:
        if gpu == gpu:
            print(f"  N={n:>6}: {16/gpu:>5.1f}x real-time")

    try:
        rtt_ms = bench_webots_rtt()
        print(f"\nOmniSim wire-protocol RTT (proxy for importMFNodeFromString "
              f"per-spawn cost under load): {rtt_ms:.2f} ms / call")
        print(f"At a typical 50-particles-per-collision-step burst:")
        for n, _, gpu, _ in rows:
            if gpu == gpu:
                ratio = (rtt_ms * 50) / gpu
                print(f"  N={n:>6}: GPU {gpu:.3f} ms vs OmniSim-spawn "
                      f"{rtt_ms*50:.0f} ms -> {ratio:.0f}x")
    except (ConnectionError, OSError) as exc:
        print(f"\nOmniSim baseline skipped: {exc}")


if __name__ == "__main__":
    sys.exit(main())
