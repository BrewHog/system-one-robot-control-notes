#!/usr/bin/env python3
"""Does JEV actually discriminate on state?

A System 1 controller is only useful if its judgment changes when the state
changes. This runs the same question against deliberately opposite states and
reports whether the answers separate.

Usage:
    python3 scripts/jev_discriminate.py
"""

from __future__ import annotations

import os
import time
from pathlib import Path

from typesafe_sdk import Noul, Score, TypeSafeClient


def load_api_key() -> str:
    key = os.environ.get("TYPESAFE_API_KEY")
    if key:
        return key
    env_file = Path(__file__).resolve().parent.parent / ".env"
    for line in env_file.read_text().splitlines():
        if line.strip().startswith("TYPESAFE_API_KEY="):
            return line.split("=", 1)[1].strip()
    raise SystemExit("TYPESAFE_API_KEY not found")


# Clearly opposite situations. A working judgment must separate these.
CASES = [
    (
        "clear path straight to goal",
        {
            "robot": {"x": 1.0, "y": 1.0, "theta": 0.0},
            "goal": {"x": 9.0, "y": 1.0},
            "obstacle": None,
        },
    ),
    (
        "goal directly behind the robot",
        {
            "robot": {"x": 1.0, "y": 1.0, "theta": 0.0},
            "goal": {"x": -5.0, "y": 1.0},
            "obstacle": None,
        },
    ),
    (
        "wall one metre ahead, goal beyond it",
        {
            "robot": {"x": 1.0, "y": 1.0, "theta": 0.0},
            "goal": {"x": 9.0, "y": 1.0},
            "obstacle": {"distance_m": 1.0, "bearing_deg": 0},
        },
    ),
]

QUESTIONS = {
    "goal_ahead": Noul(
        instructions="Is the goal located roughly straight ahead of the robot's current heading?"
    ),
    "blocked": Noul(
        instructions="Is the robot's forward path blocked by an obstacle within 2 metres?"
    ),
    "distance": Score(
        instructions="How far is the robot from its goal?",
        criteria=["touching", "a few metres", "several metres", "far away"],
    ),
}


def main() -> int:
    os.environ["TYPESAFE_API_KEY"] = load_api_key()

    rows = []
    with TypeSafeClient() as client:
        for label, state in CASES:
            started = time.perf_counter()
            response = client.system_one(state=state, questions=QUESTIONS)
            ms = (time.perf_counter() - started) * 1000
            rows.append(
                (
                    label,
                    response.nouls["goal_ahead"].noul,
                    response.nouls["blocked"].noul,
                    response.scores["distance"].score,
                    ms,
                )
            )

    header = f"{'case':<38} {'goal_ahead':>10} {'blocked':>8} {'distance':>9} {'ms':>7}"
    print(header)
    print("-" * len(header))
    for label, goal, blocked, dist, ms in rows:
        print(f"{label:<38} {goal:>10.2f} {blocked:>8.2f} {dist:>9.2f} {ms:>7.0f}")

    ahead = [r[1] for r in rows]
    blocked = [r[2] for r in rows]
    spread_ahead = max(ahead) - min(ahead)
    spread_blocked = max(blocked) - min(blocked)
    print()
    print(f"spread goal_ahead: {spread_ahead:.2f}   spread blocked: {spread_blocked:.2f}")

    # Row 0 is clear-and-ahead; row 1 is goal-behind; row 2 is blocked ahead.
    ok_ahead = rows[0][1] > rows[1][1]
    ok_blocked = rows[2][2] > rows[0][2]
    print(f"goal_ahead separates ahead-vs-behind : {ok_ahead}")
    print(f"blocked separates blocked-vs-clear   : {ok_blocked}")
    if ok_ahead and ok_blocked:
        print("OK: JEV discriminates on state")
        return 0
    print("WARNING: JEV did not discriminate as expected - investigate before wiring a controller")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
