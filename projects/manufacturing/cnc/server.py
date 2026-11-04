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

"""Serve a virtual GRBL controller as a raw TCP stream (what FluidNC calls telnet).

Inside OmniSim the `grbl_twin` controller owns the clock and calls
`controller.tick()`. Without the engine, `python server.py --machine <cfg>`
runs the same controller on an ideal plant against the wall clock, which is
enough to point a sender at it and check the protocol.

Each client gets the replies to its own lines (`ok`/`error`, `$` reports, its `?`
status), as FluidNC routes them per channel; alarms, the banner and messages not
caused by a line go to every client. A new client gets the banner alone.

Telnet option negotiation (IAC sequences) is stripped: its option bytes include 0x18,
which GRBL would read as a soft reset.
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from grbl import GrblController  # noqa: E402
import machine as machine_mod    # noqa: E402

IAC, SB, SE = 0xFF, 0xFA, 0xF0


def strip_telnet(data: bytes, state: dict) -> bytes:
    out = bytearray()
    for b in data:
        mode = state.get("mode")
        if mode == "iac":
            state["mode"] = "sb" if b == SB else ("opt" if b in (0xFB, 0xFC, 0xFD, 0xFE) else None)
            if b == IAC:
                out.append(b)          # escaped 0xFF; GRBL ignores it anyway
        elif mode == "opt":
            state["mode"] = None
        elif mode == "sb":
            if b == IAC:
                state["mode"] = "sb_iac"
        elif mode == "sb_iac":
            state["mode"] = None if b == SE else "sb"
        elif b == IAC:
            state["mode"] = "iac"
        else:
            out.append(b)
    return bytes(out)


class StreamServer:
    def __init__(self, controller: GrblController, host: str = "127.0.0.1", port: int = 2323):
        self.controller = controller
        self.host, self.port = host, port
        self._clients: List[socket.socket] = []
        self._lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._stop = threading.Event()
        controller.out = self.send

    def start(self) -> int:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(4)
        self._sock = s
        self.port = s.getsockname()[1]
        threading.Thread(target=self._accept, name="grbl-accept", daemon=True).start()
        return self.port

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            self._sock.close()
        with self._lock:
            for c in self._clients:
                c.close()
            self._clients.clear()

    def send(self, text: str, client=None) -> None:
        """To one client (the one a reply belongs to), or to every client (None)."""
        data = text.encode("ascii", "replace")
        with self._lock:
            for c in ([client] if client is not None else list(self._clients)):
                if c not in self._clients:
                    continue
                try:
                    c.sendall(data)
                except OSError:
                    self._clients.remove(c)

    def _accept(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(2.0)
            with self._lock:
                self._clients.append(conn)
            # A newly connected sender expects the banner, as on a board that resets
            # when its serial port opens.
            self.send("\r\n" + self.controller.banner + "\r\n", conn)
            threading.Thread(target=self._read, args=(conn,), daemon=True).start()

    def _read(self, conn: socket.socket) -> None:
        state: dict = {}
        conn.settimeout(None)
        while not self._stop.is_set():
            try:
                data = conn.recv(4096)
            except OSError:
                break
            if not data:
                break
            self.controller.feed(strip_telnet(data, state), conn)
        with self._lock:
            if conn in self._clients:
                self._clients.remove(conn)
        conn.close()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Virtual GRBL controller on an ideal plant (no engine).")
    p.add_argument("--machine", required=True, help="GRBL $$ dump or FluidNC config.yaml")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2323)
    p.add_argument("--tick-ms", type=float, default=5.0)
    args = p.parse_args(argv)
    m = machine_mod.load(args.machine)
    ctl = GrblController(m)
    srv = StreamServer(ctl, args.host, args.port)
    port = srv.start()
    print(f"virtual GRBL for {m.name} on {args.host}:{port}", flush=True)
    dt = args.tick_ms / 1000.0
    last = time.monotonic()
    try:
        while True:
            time.sleep(dt)
            now = time.monotonic()
            ctl.tick(now - last)
            last = now
    except KeyboardInterrupt:
        srv.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
