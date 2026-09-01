#!/usr/bin/env python3
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

"""Expose a Windows-hosted OmniSim harness inside WSL2, without admin rights.

WHY THIS EXISTS
---------------
OmniSim's engine is Windows-primary; ROS 2 is Linux. The natural pairing on a
Windows workstation is "engine + harness on Windows, ROS 2 in WSL2". That pairing
has one obstacle: **WSL2 cannot open a TCP connection to a service on the Windows
host** unless someone adds a Windows Firewall / Hyper-V firewall rule, which needs
Administrator. The reverse direction — Windows dialling *into* WSL2 — works out of
the box.

So this tool inverts the direction. The WSL half listens; the Windows half dials
in and keeps a pool of idle connections parked. Each idle connection is a
pre-authorised return path: when a ROS 2 node inside WSL opens an HTTP connection
to the local listener, the WSL half hands it one of the parked connections and
pumps bytes, and the Windows half splices its end onto the real harness at
127.0.0.1:<harness-port>.

It is a byte pump, not an HTTP proxy: it never parses, buffers or rewrites the
stream, so screenshots, chunked responses and every future harness endpoint work
unchanged. One inbound HTTP connection maps to exactly one tunnel connection,
which both halves close when either side reaches EOF.

NOT REQUIRED when the harness and ROS 2 are on the same host (a Linux box, or a
Windows box where you have added a firewall rule). Point the nodes straight at the
harness with `--ros-args -p harness_url:=http://<host>:6789` instead.

USAGE
-----
Terminal A (inside WSL)::

    python3 tools/wsl_harness_link.py wsl --listen-port 6789 --tunnel-port 7799

Terminal B (on Windows)::

    python tools\\wsl_harness_link.py windows --harness-port 6889 \\
        --wsl-host 192.168.209.179 --tunnel-port 7799

`--wsl-host` is what `hostname -I` prints inside WSL (it changes on every WSL
restart; `wsl -d <distro> -- hostname -I` reads it from Windows).

Then, inside WSL, `http://127.0.0.1:6789` is the Windows harness.

⚠ THROUGHPUT CEILING -- READ THIS BEFORE RAISING A PUBLISH RATE
---------------------------------------------------------------
Since 2026-09-01 the harness AND the bridges speak **HTTP/1.1 with
keep-alive** (`protocol_version = "HTTP/1.1"`), so a pooled client reuses one
TCP connection and the ceiling below no longer applies to them. It still
applies verbatim to any client that opens a fresh connection per request
(curl in a loop, `urllib.request.urlopen`), and through this tunnel each such
request costs **two** connections on the Windows side (the dial into WSL,
plus the connect to the harness on loopback) -- so pool your connections.

Windows' default dynamic port range is 16384 ports (49152-65535) and its default
`TcpTimedWaitDelay` is 120 s, so the sustainable ceiling is::

    16384 / 120 s  ~=  136 new connections/sec  ~=  ~65 HTTP requests/sec
                                                   through this tunnel

Measured 2026-08-17: running the bringup at 50 Hz clock + 10 Hz state + 20 Hz odom
drove Windows to **17,487 sockets in TIME_WAIT** -- past the whole range -- and
`connect()` then failed with `WinError 10048` ("Only one usage of each socket
address..."), which reads like a bind conflict but is exhaustion. The symptom is
nodes reporting the harness as unreachable while it is perfectly healthy.

`omnisim_bringup.launch.py`'s defaults (~45 requests/s) sit inside this budget.
Lower them further if you run other HTTP clients against the same host. On a
same-host Linux deployment the tunnel is not involved and the budget doubles.
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import threading
import time

# Bytes moved per read(). 64 KiB keeps screenshot transfers off the syscall floor
# without pinning a large buffer per in-flight connection.
CHUNK = 65536

# How long a parked tunnel connection may sit idle before we recycle it. Parked
# connections are cheap, but a stale one whose peer died silently would otherwise
# be handed a live request and swallow it.
PARK_TIMEOUT_S = 300.0


def _pump(src: socket.socket, dst: socket.socket) -> None:
    """Copy src -> dst until src reaches EOF, then half-close dst.

    Half-closing (rather than closing) lets the opposite direction finish
    draining, which is what makes a request/response pair survive a client that
    shuts down its write side as soon as the request is out.
    """
    try:
        while True:
            chunk = src.recv(CHUNK)
            if not chunk:
                break
            dst.sendall(chunk)
    except OSError:
        pass
    finally:
        try:
            dst.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def _splice(a: socket.socket, b: socket.socket) -> None:
    """Pump a<->b concurrently, returning once both directions are done."""
    t = threading.Thread(target=_pump, args=(a, b), daemon=True)
    t.start()
    _pump(b, a)
    t.join(timeout=10.0)
    for sock in (a, b):
        try:
            sock.close()
        except OSError:
            pass


# --------------------------------------------------------------------------
# WSL half: listens for ROS 2 clients, and for the Windows agent dialling in.
# --------------------------------------------------------------------------


def _peer_closed(sock: socket.socket) -> bool:
    """True when the peer has already closed this parked connection.

    A parked connection carries no data until it is activated, so anything
    readable on it means EOF. ``MSG_PEEK`` keeps the check non-destructive in the
    (impossible-by-protocol, but cheap to guard) case that real bytes arrived.
    """
    try:
        readable, _, errored = select.select([sock], [], [sock], 0)
    except (OSError, ValueError):
        return True
    if errored:
        return True
    if not readable:
        return False
    try:
        return sock.recv(1, socket.MSG_PEEK) == b""
    except OSError:
        return True


class _Pool:
    """Parked connections from the Windows half, oldest-first."""

    def __init__(self) -> None:
        self._items: list[tuple[float, socket.socket]] = []
        self._cv = threading.Condition()

    def put(self, sock: socket.socket) -> None:
        with self._cv:
            self._items.append((time.monotonic(), sock))
            self._cv.notify()

    def get(self, timeout: float) -> socket.socket | None:
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                # Drop anything parked past its welcome before handing one out.
                now = time.monotonic()
                while self._items and now - self._items[0][0] > PARK_TIMEOUT_S:
                    _, stale = self._items.pop(0)
                    try:
                        stale.close()
                    except OSError:
                        pass
                # ⚠ AND DROP DEAD ONES. If the Windows half is restarted, its old
                # parked connections stay in this pool as half-closed sockets.
                # Handing one to a request fails that request instantly, and the
                # Windows half -- seeing its parked socket close the moment it was
                # activated -- immediately dials a replacement. That feedback loop
                # is a connection storm: measured, it produced ~174 connections/sec
                # against a ~13 request/sec workload and exhausted the Windows
                # ephemeral port range in about 90 seconds.
                #
                # A peer that has closed shows up as *readable with zero bytes*, so
                # a zero-timeout select distinguishes "dead" from "parked and idle"
                # without consuming a live request.
                while self._items and _peer_closed(self._items[0][1]):
                    _, dead = self._items.pop(0)
                    try:
                        dead.close()
                    except OSError:
                        pass
                if self._items:
                    return self._items.pop(0)[1]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cv.wait(remaining)

    def size(self) -> int:
        with self._cv:
            return len(self._items)


def run_wsl(listen_host: str, listen_port: int, tunnel_host: str, tunnel_port: int) -> int:
    pool = _Pool()

    def accept_tunnel() -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((tunnel_host, tunnel_port))
        srv.listen(64)
        print(f"[link/wsl] tunnel listening on {tunnel_host}:{tunnel_port}", flush=True)
        while True:
            conn, _ = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            pool.put(conn)

    threading.Thread(target=accept_tunnel, daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((listen_host, listen_port))
    srv.listen(64)
    print(
        f"[link/wsl] harness proxy listening on http://{listen_host}:{listen_port}",
        flush=True,
    )

    def serve(client: socket.socket) -> None:
        tunnel = pool.get(timeout=15.0)
        if tunnel is None:
            # Answer in-band rather than dropping the connection, so the caller
            # sees why instead of a bare ConnectionReset.
            client.sendall(
                b"HTTP/1.1 502 Bad Gateway\r\n"
                b"Content-Type: application/json\r\n"
                b"Connection: close\r\n\r\n"
                b'{"error":"no tunnel connection from the Windows half; '
                b'start tools/wsl_harness_link.py windows"}'
            )
            client.close()
            return
        _splice(client, tunnel)

    while True:
        client, _ = srv.accept()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=serve, args=(client,), daemon=True).start()


# --------------------------------------------------------------------------
# Windows half: dials WSL, parks, and splices activated connections onto the
# local harness.
# --------------------------------------------------------------------------


def run_windows(
    wsl_host: str, tunnel_port: int, harness_host: str, harness_port: int, pool_size: int
) -> int:
    print(
        f"[link/windows] dialling {wsl_host}:{tunnel_port}, "
        f"forwarding to {harness_host}:{harness_port} (pool={pool_size})",
        flush=True,
    )
    live = threading.Semaphore(pool_size)
    # Backoff state, shared across dialling threads. Without it, a host that has
    # run out of ephemeral ports gets `pool_size` failed connect() attempts per
    # second for ever -- a retry storm that consumes the very resource it is
    # waiting for and prevents the recovery it is polling for. Measured: this
    # spammed 1400 identical WinError 10048 lines and kept TIME_WAIT pinned.
    backoff = {"s": 0.0, "reported": ""}
    backoff_lock = threading.Lock()

    def one_connection() -> None:
        with backoff_lock:
            wait = backoff["s"]
        if wait:
            time.sleep(wait)
        try:
            tunnel = socket.create_connection((wsl_host, tunnel_port), timeout=10.0)
        except OSError as exc:
            with backoff_lock:
                backoff["s"] = min(max(backoff["s"] * 2.0, 0.25), 5.0)
                text = str(exc)
                if text != backoff["reported"]:
                    backoff["reported"] = text
                    print(
                        f"[link/windows] dial failed ({text}); "
                        f"backing off to {backoff['s']:.2f}s. If this is "
                        f"WinError 10048, the host is out of ephemeral ports -- "
                        f"lower your ROS publish rates (see the module docstring).",
                        flush=True,
                    )
            live.release()
            return
        with backoff_lock:
            if backoff["s"]:
                print("[link/windows] dialling recovered", flush=True)
            backoff["s"] = 0.0
            backoff["reported"] = ""
        tunnel.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        tunnel.settimeout(None)
        parked_at = time.monotonic()
        try:
            # Park until the WSL half routes a request to us. The first byte of
            # that request is the activation signal.
            first = tunnel.recv(CHUNK)
        except OSError:
            first = b""
        # Free the pool slot the moment we stop being idle, so a replacement is
        # dialled while this request is still in flight.
        live.release()
        if not first:
            try:
                tunnel.close()
            except OSError:
                pass
            # A connection that died almost as soon as it was parked means the
            # far side is churning, not serving. Re-dialling instantly turns that
            # into a storm, so borrow the dial backoff. A connection that parked
            # for a while and then closed is an ordinary idle timeout: no backoff.
            if time.monotonic() - parked_at < 0.25:
                with backoff_lock:
                    backoff["s"] = min(max(backoff["s"] * 2.0, 0.25), 5.0)
            return
        try:
            upstream = socket.create_connection((harness_host, harness_port), timeout=10.0)
        except OSError as exc:
            try:
                tunnel.sendall(
                    b"HTTP/1.1 502 Bad Gateway\r\n"
                    b"Content-Type: application/json\r\n"
                    b"Connection: close\r\n\r\n"
                    + f'{{"error":"harness unreachable at '
                    f'{harness_host}:{harness_port}: {exc}"}}'.encode()
                )
                tunnel.close()
            except OSError:
                pass
            # ⚠ BACK OFF HERE TOO. A dead upstream is the other storm path: the
            # slot was already released, so without this the pool re-dials
            # instantly, fails upstream again, and burns two ephemeral ports per
            # cycle. Measured: when the harness process exited, this loop refilled
            # the Windows TIME_WAIT table (16k sockets) within minutes, so the
            # tunnel could not recover even after the harness came back.
            with backoff_lock:
                backoff["s"] = min(max(backoff["s"] * 2.0, 0.25), 5.0)
                text = f"upstream {harness_host}:{harness_port}: {exc}"
                if text != backoff["reported"]:
                    backoff["reported"] = text
                    print(f"[link/windows] {text}; backing off to {backoff['s']:.2f}s",
                          flush=True)
            return
        upstream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        upstream.sendall(first)
        _splice(tunnel, upstream)

    while True:
        live.acquire()
        threading.Thread(target=one_connection, daemon=True).start()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="side", required=True)

    w = sub.add_parser("wsl", help="run inside WSL: listen for ROS 2 clients")
    w.add_argument("--listen-host", default="127.0.0.1")
    w.add_argument("--listen-port", type=int, default=6789)
    w.add_argument("--tunnel-host", default="0.0.0.0")
    w.add_argument("--tunnel-port", type=int, default=7799)

    n = sub.add_parser("windows", help="run on Windows: dial WSL and splice to the harness")
    n.add_argument("--wsl-host", required=True, help="WSL IP, from `hostname -I` inside WSL")
    n.add_argument("--tunnel-port", type=int, default=7799)
    n.add_argument("--harness-host", default="127.0.0.1")
    n.add_argument("--harness-port", type=int, default=6789)
    n.add_argument("--pool-size", type=int, default=8)

    args = ap.parse_args(argv)
    try:
        if args.side == "wsl":
            return run_wsl(
                args.listen_host, args.listen_port, args.tunnel_host, args.tunnel_port
            )
        return run_windows(
            args.wsl_host, args.tunnel_port, args.harness_host, args.harness_port, args.pool_size
        )
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
