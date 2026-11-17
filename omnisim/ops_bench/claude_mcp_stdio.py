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
"""Stdio MCP server that exposes the bridge tools to a native Claude Code session.

Standard library only. Claude Code spawns it; it lists the tool schemas it was
given and forwards every call to the benchmark adapter's loopback endpoint,
which applies the same gating, cancellation and logging as the Codex arm.
It never talks to the robot itself.
"""
import json
import os
import sys
import threading
import urllib.request

TOOLS = json.loads(open(os.environ["OPS_BENCH_TOOLS_FILE"], encoding="utf-8").read())
ENDPOINT = os.environ["OPS_BENCH_TOOL_ENDPOINT"]
TOKEN = os.environ["OPS_BENCH_TOOL_TOKEN"]
out_lock = threading.Lock()


def send(message):
    with out_lock:
        sys.stdout.write(json.dumps(message, ensure_ascii=False) + "\n")
        sys.stdout.flush()


def call(ident, params):
    try:
        body = json.dumps({"name": params.get("name"), "arguments": params.get("arguments") or {},
                           "token": TOKEN}).encode("utf-8")
        request = urllib.request.Request(ENDPOINT, body, {"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=170) as response:
            result = json.loads(response.read().decode("utf-8"))
        send({"jsonrpc": "2.0", "id": ident, "result": {
            "content": [{"type": "text", "text": result["text"]}], "isError": not result["success"]}})
    except Exception as exc:
        send({"jsonrpc": "2.0", "id": ident, "result": {
            "content": [{"type": "text", "text": json.dumps({"error": f"{type(exc).__name__}: {exc}"})}],
            "isError": True}})


def main():
    for line in sys.stdin:
        try:
            message = json.loads(line)
        except ValueError:
            continue
        method, ident = message.get("method"), message.get("id")
        if ident is None:
            continue  # notifications (initialized, cancelled): the adapter owns cancellation
        if method == "initialize":
            send({"jsonrpc": "2.0", "id": ident, "result": {
                "protocolVersion": message.get("params", {}).get("protocolVersion", "2025-06-18"),
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "robot", "version": "0.1.0"}}})
        elif method == "tools/list":
            send({"jsonrpc": "2.0", "id": ident, "result": {"tools": [
                {"name": t["name"], "description": t["description"],
                 # MCP requires an object schema; an empty one means "no arguments".
                 "inputSchema": {"type": "object", "properties": {}, **(t["parameters"] or {})}}
                for t in TOOLS]}})
        elif method == "tools/call":
            threading.Thread(target=call, args=(ident, message.get("params", {})), daemon=True).start()
        elif method == "ping":
            send({"jsonrpc": "2.0", "id": ident, "result": {}})
        else:
            send({"jsonrpc": "2.0", "id": ident,
                  "error": {"code": -32601, "message": f"Method not found: {method}"}})


if __name__ == "__main__":
    main()
