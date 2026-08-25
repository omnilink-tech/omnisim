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

"""Tests for HTTP connection reuse in the client transport.

WHY THIS IS WORTH A TEST
------------------------
The cost being defended against is not latency, it is **sockets**. Every closed
client socket sits in ``TIME_WAIT`` for 120 s on Windows against a 16,384-port
ephemeral range, and a ~50 Hz ROS 2 bringup measured 17,487 of them before
``connect()`` began failing with ``WinError 10048``. So the property under test
is "one connection for many requests", asserted by counting the distinct client
ports the server saw -- not by timing anything.

Measured against the live bridge (2026-08-17, machine ``9722d23d12a3``):
300 requests cost 300 TIME_WAIT sockets at 114.8 req/s without reuse, and
0 sockets at 908.8 req/s with it.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from omnisim_ros2.harness_client import HarnessClient

CLIENT_PORTS = []
CLOSE_EVERY_TIME = {"on": False}


class _KeepAliveHandler(BaseHTTPRequestHandler):
    # The whole point: without this the server closes after every response and
    # no amount of client pooling helps.
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def _reply(self):
        length = int(self.headers.get("Content-Length") or 0)
        if length:
            self.rfile.read(length)
        CLIENT_PORTS.append(self.client_address[1])
        body = json.dumps({"ok": True}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        if CLOSE_EVERY_TIME["on"]:
            # Simulate a server that hangs up after each response, which is
            # what an idle keep-alive connection eventually does for real.
            self.send_header("Connection", "close")
            self.close_connection = True
        self.end_headers()
        self.wfile.write(body)

    do_GET = _reply
    do_POST = _reply


@pytest.fixture()
def server_url():
    CLIENT_PORTS.clear()
    CLOSE_EVERY_TIME["on"] = False
    server = ThreadingHTTPServer(("127.0.0.1", 0), _KeepAliveHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    host, port = server.server_address
    yield f"http://{host}:{port}"
    server.shutdown()


def test_keep_alive_reuses_one_connection(server_url):
    c = HarnessClient(server_url, timeout_s=5, keep_alive=True)
    for _ in range(10):
        assert c.get("/x").ok
    c.close()
    assert len(CLIENT_PORTS) == 10
    assert len(set(CLIENT_PORTS)) == 1, (
        f"expected one pooled connection, saw {len(set(CLIENT_PORTS))}"
    )


def test_disabled_keep_alive_uses_a_connection_per_request(server_url):
    """The A/B arm, and the pre-2026-08-17 behaviour."""
    c = HarnessClient(server_url, timeout_s=5, keep_alive=False)
    for _ in range(10):
        assert c.get("/x").ok
    c.close()
    assert len(set(CLIENT_PORTS)) == 10


def test_server_that_closes_each_response_still_works(server_url):
    """An HTTP/1.0 peer (the World Harness today) must degrade, not break."""
    CLOSE_EVERY_TIME["on"] = True
    c = HarnessClient(server_url, timeout_s=5, keep_alive=True)
    for _ in range(5):
        assert c.get("/x").ok
    c.close()
    # Correctness is that all five succeeded; each necessarily used a new
    # socket because the server hung up every time.
    assert len(CLIENT_PORTS) == 5
    assert len(set(CLIENT_PORTS)) == 5


def test_pooled_connection_recovers_after_the_peer_hangs_up(server_url):
    """A stale pooled connection must be retried transparently.

    This is the failure that would otherwise surface as a random dead node:
    the server closes an idle keep-alive socket, and the next publish tick
    writes into it.
    """
    c = HarnessClient(server_url, timeout_s=5, keep_alive=True)
    assert c.get("/x").ok
    # Kill the pooled socket behind the client's back.
    c._conn.sock.close()
    # The retry path must reconnect rather than raise out of a timer callback.
    assert c.get("/x").ok
    c.close()


def test_close_is_idempotent(server_url):
    c = HarnessClient(server_url, timeout_s=5, keep_alive=True)
    c.get("/x")
    c.close()
    c.close()
    assert c.get("/x").ok
    c.close()
