#!/usr/bin/env python3
"""Drive the simulated robot through ros-mcp to produce a visually obvious path.

Turns and moves so the rendered trail is unmistakable in a screenshot, and
prints the pose before/after so the motion is independently checkable.

Usage:
    python3 scripts/sim_demo_motion.py [--port 9090] [--shape square|arc|zigzag]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path

import websocket
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parent.parent
ROS_MCP = ROOT / "vendor/ros-mcp-server/.venv/bin/ros-mcp"


def read_pose(port: int, timeout: float = 10.0) -> dict:
    ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=timeout)
    ws.send(
        json.dumps(
            {
                "op": "subscribe",
                "topic": "/turtle1/pose",
                "type": "turtlesim/msg/Pose",
                "id": "pose_probe",
            }
        )
    )
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            data = json.loads(ws.recv())
            if data.get("op") == "publish":
                return data["msg"]
    finally:
        ws.close()
    return {}


def twist(linear: float, angular: float) -> dict:
    return {
        "linear": {"x": linear, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular},
    }


# (linear, angular, seconds) - a square traced from the centre.
SQUARE = [
    (1.5, 0.0, 1.6),
    (0.0, 1.6, 1.55),
    (1.5, 0.0, 1.6),
    (0.0, 1.6, 1.55),
    (1.5, 0.0, 1.6),
    (0.0, 1.6, 1.55),
    (1.5, 0.0, 1.6),
]

ARC = [
    (0.0, 1.2, 1.0),
    (1.8, 0.9, 3.2),
    (0.0, -1.4, 0.9),
    (1.6, 0.0, 1.2),
]


async def run(port: int, shape: str) -> int:
    script = SQUARE if shape == "square" else ARC

    before = read_pose(port)
    print(f"pose before: x={before.get('x'):.2f} y={before.get('y'):.2f} "
          f"theta={before.get('theta'):.2f}")

    params = StdioServerParameters(command=str(ROS_MCP), args=["--transport=stdio"])
    # ros-mcp logs connection noise to stderr; keep it out of the report.
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            await session.call_tool("connect_to_robot", {"ip": "127.0.0.1", "port": port})
            print(f"connected to rosbridge on 127.0.0.1:{port}")
            print(f"drawing '{shape}' via {len(script)} MCP publish_once calls...")

            for i, (linear, angular, duration) in enumerate(script, 1):
                await session.call_tool(
                    "publish_once",
                    {
                        "topic": "/turtle1/cmd_vel",
                        "msg_type": "geometry_msgs/msg/Twist",
                        "msg": twist(linear, angular),
                    },
                )
                await asyncio.sleep(duration)
                print(f"  segment {i}/{len(script)} done")

            # Stop the robot.
            await session.call_tool(
                "publish_once",
                {
                    "topic": "/turtle1/cmd_vel",
                    "msg_type": "geometry_msgs/msg/Twist",
                    "msg": twist(0.0, 0.0),
                },
            )

    await asyncio.sleep(0.4)
    after = read_pose(port)
    print(f"pose after : x={after.get('x'):.2f} y={after.get('y'):.2f} "
          f"theta={after.get('theta'):.2f}")

    moved = math.hypot(after.get("x", 0) - before.get("x", 0), after.get("y", 0) - before.get("y", 0))
    print(f"nett displacement: {moved:.2f} m")
    if moved < 0.3:
        print("FAIL: robot did not move", file=sys.stderr)
        return 1
    print("OK: robot moved under MCP control")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--shape", choices=["square", "arc"], default="square")
    args = parser.parse_args()
    return asyncio.run(run(args.port, args.shape))


if __name__ == "__main__":
    raise SystemExit(main())
