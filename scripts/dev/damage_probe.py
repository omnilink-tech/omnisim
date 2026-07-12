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

"""damage_probe — one-shot debug: connect to a running harness and dump damage_state.

Use this when damage_events_capture returns 0 events to diagnose whether
the damage tracker is initialised, what robot it's tracking, and what
the current part-state snapshot looks like.

  python scripts/dev/damage_probe.py
"""
import json, os, socket, struct, sys

HOST = os.environ.get("OMNISIM_HARNESS_SUPERVISOR_HOST", "127.0.0.1")
PORT = int(os.environ.get("OMNISIM_HARNESS_SUPERVISOR_PORT", "6790"))


def _recv(sock, n):
    out = b""
    while len(out) < n:
        c = sock.recv(n - len(out))
        if not c:
            return None
        out += c
    return out


def _req(sock, cmd, args=None):
    body = json.dumps({"cmd": cmd, "args": args or {}}).encode()
    sock.sendall(struct.pack(">I", len(body)) + body)
    h = _recv(sock, 4)
    if not h:
        return None
    (n,) = struct.unpack(">I", h)
    payload = _recv(sock, n)
    return json.loads(payload.decode())


def main():
    s = socket.create_connection((HOST, PORT), timeout=5.0)
    s.settimeout(5.0)
    for cmd in ("ping", "sim_state", "damage_state", "damage_geometry_stats",
                ("damage_events", {"since": 0, "limit": 5})):
        if isinstance(cmd, tuple):
            cmd, args = cmd
        else:
            args = None
        try:
            r = _req(s, cmd, args)
            print(f"--- {cmd} ---")
            print(json.dumps(r, indent=2, default=str)[:2000])
        except Exception as e:
            print(f"--- {cmd} FAILED: {e!r} ---")
    s.close()


if __name__ == "__main__":
    main()
