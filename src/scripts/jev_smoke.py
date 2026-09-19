#!/usr/bin/env python3
"""Smoke test: confirm the JEV (TypeSafe System One) credentials work.

Reads TYPESAFE_API_KEY from the environment (or .env). Measures wall-clock
latency, because the whole reason JEV matters here is round-trip speed.

Usage:
    TYPESAFE_API_KEY=... python3 scripts/jev_smoke.py
"""

from __future__ import annotations

import os
import time
from pathlib import Path

from typesafe_sdk import Choice, Noul, Score, TypeSafeClient


def load_api_key() -> str:
    key = os.environ.get("TYPESAFE_API_KEY")
    if key:
        return key
    env_file = Path(__file__).resolve().parent.parent / ".env"
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if line.startswith("TYPESAFE_API_KEY="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    raise SystemExit("TYPESAFE_API_KEY not set (env or .env)")


def main() -> int:
    os.environ["TYPESAFE_API_KEY"] = load_api_key()

    # A robotics-flavoured question set: the shape a System 1 controller needs.
    state = {
        "robot": {
            "pose": {"x": 5.54, "y": 5.54, "theta": 0.0},
            "goal": {"x": 8.0, "y": 5.5},
            "last_action": "rotate left",
            "collision": False,
        },
        "observation": "The turtle is facing away from the goal with clear space ahead.",
    }

    with TypeSafeClient() as client:
        started = time.perf_counter()
        response = client.system_one(
            state=state,
            questions={
                "goal_ahead": Noul(
                    instructions="Is the robot's goal roughly straight ahead of its current heading?"
                ),
                "turn": Choice(
                    instructions="Which way should the robot rotate?",
                    criteria={"left": None, "right": None, "neither": None},
                ),
                "progress": Score(
                    instructions="How far along is the robot toward its goal?",
                    criteria=["just started", "making progress", "nearly there", "arrived"],
                ),
            },
        )
        elapsed_ms = (time.perf_counter() - started) * 1000

    print(f"round-trip: {elapsed_ms:.0f} ms")
    print(f"goal_ahead : {response.nouls['goal_ahead'].noul}")
    print(f"turn       : {response.choices['turn'].choice}")
    print(f"progress   : {response.scores['progress'].score}")
    print("OK: JEV credentials work")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
