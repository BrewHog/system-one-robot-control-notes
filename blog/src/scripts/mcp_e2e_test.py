#!/usr/bin/env python3
"""End-to-end proof: MCP client -> ros-mcp server -> rosbridge -> simulated turtle.

Spawns the ros-mcp server over stdio exactly like an LLM client would, drives the
turtle through the MCP protocol, and verifies the simulated robot actually moved
by reading /turtle1/pose over a second rosbridge connection.

Usage:
    python3 scripts/mcp_e2e_test.py [--port 9090] [--ros-mcp /path/to/ros-mcp]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time

import websocket
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


def read_pose(port: int, timeout: float = 10.0) -> dict:
    """Subscribe to /turtle1/pose briefly and return the first message."""
    ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=timeout)
    ws.send(
        json.dumps(
            {
                "op": "subscribe",
                "topic": "/turtle1/pose",
                "type": "turtlesim/msg/Pose",
                "throttle_rate": 0,
                "queue_length": 0,
                "id": "pose_probe",
            }
        )
    )
    pose = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        data = json.loads(ws.recv())
        if data.get("op") == "publish" and data.get("topic") == "/turtle1/pose":
            pose = data["msg"]
            break
    ws.close()
    return pose


def extract(result) -> object:
    """Pull the JSON payload out of an MCP tool result."""
    structured = getattr(result, "structuredContent", None)
    if structured is not None:
        return structured
    for block in getattr(result, "content", []) or []:
        text = getattr(block, "text", None)
        if text:
            try:
                return json.loads(text)
            except json.JSONDecodeError:
                return text
    return None


async def run(port: int, ros_mcp_cmd: list[str]) -> int:
    failures: list[str] = []

    before = read_pose(port)
    print(f"pose before : {before}")
    if before is None:
        print("FAIL: could not read /turtle1/pose - is the simulator running?", file=sys.stderr)
        return 1

    params = StdioServerParameters(command=ros_mcp_cmd[0], args=ros_mcp_cmd[1:])
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            tools = await session.list_tools()
            names = sorted(t.name for t in tools.tools)
            print(f"tools exposed by ros-mcp ({len(names)}): {', '.join(names)}")
            for required in ("connect_to_robot", "get_topics", "publish_once"):
                if required not in names:
                    failures.append(f"missing expected tool: {required}")

            conn = extract(
                await session.call_tool("connect_to_robot", {"ip": "127.0.0.1", "port": port})
            )
            print(f"connect_to_robot -> {json.dumps(conn)[:220]}")

            topics = extract(await session.call_tool("get_topics", {}))
            topic_list = topics.get("topics", []) if isinstance(topics, dict) else []
            print(f"get_topics -> {len(topic_list)} topics")
            if "/turtle1/cmd_vel" not in topic_list:
                failures.append("/turtle1/cmd_vel not discovered by get_topics")

            # Drive the robot through MCP: turn in place, then go forward.
            await session.call_tool(
                "publish_once",
                {
                    "topic": "/turtle1/cmd_vel",
                    "msg_type": "geometry_msgs/msg/Twist",
                    "msg": {
                        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 1.6},
                    },
                },
            )
            await asyncio.sleep(1.4)
            await session.call_tool(
                "publish_once",
                {
                    "topic": "/turtle1/cmd_vel",
                    "msg_type": "geometry_msgs/msg/Twist",
                    "msg": {
                        "linear": {"x": 1.6, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                    },
                },
            )
            await asyncio.sleep(1.6)

    after = read_pose(port)
    print(f"pose after  : {after}")
    if after is None:
        failures.append("could not read pose after driving")
    else:
        dx = after["x"] - before["x"]
        dy = after["y"] - before["y"]
        moved = math.hypot(dx, dy)
        turned = after["theta"] - before["theta"]
        print(f"delta       : dx={dx:+.3f} dy={dy:+.3f} distance={moved:.3f} dtheta={turned:+.3f}")
        if moved < 0.2:
            failures.append(f"turtle barely moved ({moved:.3f} m) - command path may be broken")

    print()
    if failures:
        for f in failures:
            print(f"FAIL: {f}")
        return 1
    print("PASS: MCP client drove the simulated robot through ros-mcp")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument(
        "--ros-mcp",
        default=str(
            __import__("pathlib").Path(__file__).resolve().parent.parent
            / "vendor/ros-mcp-server/.venv/bin/ros-mcp"
        ),
        help="path to the ros-mcp executable",
    )
    parser.add_argument("--transport-arg", default="--transport=stdio")
    args = parser.parse_args()

    cmd = [args.ros_mcp, args.transport_arg]
    return asyncio.run(run(args.port, cmd))


if __name__ == "__main__":
    raise SystemExit(main())
