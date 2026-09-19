#!/usr/bin/env python3
"""Raw rosbridge probe: prove the simulator answers ROS introspection calls.

Talks to rosbridge over the same WebSocket protocol ros-mcp uses, without any
ROS installation on the host. Prints the topics/services the simulator exposes.

Usage:
    python3 scripts/probe_rosbridge.py [--port 9090]
"""

from __future__ import annotations

import argparse
import json
import sys

import websocket


def call_service(ws, service: str, args=None, timeout: float = 5.0):
    """Send one rosapi service call and return the decoded result."""
    msg = {
        "op": "call_service",
        "service": service,
        "args": args or {},
        "id": f"probe_{service.replace('/', '_')}",
    }
    ws.send(json.dumps(msg))
    while True:
        raw = ws.recv()
        data = json.loads(raw)
        if data.get("op") == "service_response" or "values" in data:
            if data.get("result") is False:
                return {"error": data.get("values") or data}
            return data.get("values", data)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()

    url = f"ws://{args.host}:{args.port}"
    print(f"connecting to {url} ...")
    ws = websocket.create_connection(url, timeout=10)

    version = call_service(ws, "/rosapi/get_ros_version")
    print(f"ROS version: {version}")

    topics = call_service(ws, "/rosapi/topics")
    names = topics.get("topics", []) if isinstance(topics, dict) else []
    print(f"topics ({len(names)}):")
    for name in sorted(names):
        print(f"  {name}")

    services = call_service(ws, "/rosapi/services")
    svc_names = services.get("services", []) if isinstance(services, dict) else []
    print(f"services: {len(svc_names)}")

    nodes = call_service(ws, "/rosapi/nodes")
    print(f"nodes: {nodes}")

    ws.close()

    if "/turtle1/cmd_vel" not in names:
        print("FAIL: /turtle1/cmd_vel not exposed by the simulator", file=sys.stderr)
        return 1
    print("OK: simulator is reachable and exposing turtle control topics")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
