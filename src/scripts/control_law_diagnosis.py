#!/usr/bin/env python3
"""Diagnose the re-orientation limit cycle, then demonstrate a converging fix.

The bug under test
------------------
`jev_controller.py` chooses the turn between judgments (the "slow loop") but
then holds that choice open for `--hold` seconds of continuous actuation while
re-deriving a *proportional* angular command from live pose inside the "fast
loop":

    if abs(error) > 0.5:            # coarse re-orientation deadband
        linear = 0.0
        angular = clamp(2.4 * error, -2.2, 2.2)

The two gains are not matched to each other. With hold=0.9s and the angular
saturated at 2.2 rad/s, one hold can rotate the robot by up to 1.98 rad while
the deadband it is trying to enter is only 0.5 rad wide. So the robot sails
straight through the deadband, `error` changes sign, and it re-enters the same
branch pointing the other way. Diagonal motion never starts.

This script:
  1. RESET  - puts the sim at a known pose, measures the initial error.
  2. BUG    - replays the shipped law and records |error| over time.
  3. FIX    - replays a rate-limited variant and records |error| over time.
  4. reports whether each converges.

The fix keeps the same architecture - JEV still owns the judgment, code still
owns the control law. It only:
  * rate-limits the turn so one hold can never overshoot the deadband, and
  * creeps forward while turning, so a target behind the robot cannot deadlock.

Usage:
    python3 scripts/control_law_diagnosis.py [--port 9090] [--hold 0.9]
"""

from __future__ import annotations

import argparse
import json
import math
import time

import websocket

# Match the constants in jev_controller.py so this is a true replay.
MIN_LINEAR = 0.0
MAX_LINEAR = 2.2
ACTUATION_HZ = 20.0
COARSE_DEADBAND = 0.5  # rad, must match jev_controller.py
ANG_GAIN_COARSE = 2.4
ANG_LIMIT_COARSE = 2.2
ANG_GAIN_FINE = 1.4
ANG_LIMIT_FINE = 1.8

TARGET = {"name": "north-east shelf", "x": 8.6, "y": 8.6}


def norm_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def distance_to_walls(x: float, y: float) -> float:
    return min(x, y, 11.09 - x, 11.09 - y)


class Rosbridge:
    def __init__(self, port: int) -> None:
        self.ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=10)
        self._id = 0
        self.ws.send(
            json.dumps(
                {
                    "op": "subscribe",
                    "topic": "/turtle1/pose",
                    "type": "turtlesim/msg/Pose",
                    "id": "pose",
                }
            )
        )

    def _next(self) -> str:
        self._id += 1
        return f"d{self._id}"

    def pose(self) -> dict:
        """Return the freshest pose, skipping queued backlog.

        Reading only the next message would return a stale frame that was already
        in flight before the last command, which makes the diagnosis report lag
        rather than behaviour. Drain to the newest frame instead.
        """
        self.ws.settimeout(0.25)
        newest = None
        for _ in range(200):
            try:
                data = json.loads(self.ws.recv())
            except Exception:  # noqa: BLE001 - timeout means the queue is empty
                break
            if data.get("op") == "publish" and data.get("topic") == "/turtle1/pose":
                newest = data["msg"]
        if newest is None:
            raise TimeoutError("no pose")
        return newest

    def publish(self, linear: float, angular: float) -> None:
        self.ws.send(
            json.dumps(
                {
                    "op": "publish",
                    "topic": "/turtle1/cmd_vel",
                    "msg": {
                        "linear": {"x": linear, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": angular},
                    },
                    "id": self._next(),
                }
            )
        )

    def reset(self) -> None:
        self.ws.send(
            json.dumps({"op": "call_service", "service": "/reset", "args": {}, "id": self._next()})
        )
        self.ws.settimeout(5)
        for _ in range(50):
            if json.loads(self.ws.recv()).get("op") == "service_response":
                return
        raise TimeoutError("reset did not answer")

    def close(self) -> None:
        try:
            self.publish(0.0, 0.0)
        finally:
            self.ws.close()


def shipped_law(error: float, urgency: float, wall_risk: float, distance: float, fixed: bool) -> tuple[float, float]:
    """Return (linear, angular) for one actuation step.

    `fixed=False` reproduces jev_controller.py exactly.
    `fixed=True`  applies the rate limit + forward creep.
    """
    linear = MIN_LINEAR + urgency * (MAX_LINEAR - MIN_LINEAR)
    if wall_risk > 0.6:
        linear = min(linear, 0.45)
    if distance < 0.8:
        linear = min(linear, 0.5)

    if abs(error) > COARSE_DEADBAND:
        if not fixed:
            # --- the shipped law ---
            linear = 0.0
            angular = max(-ANG_LIMIT_COARSE, min(ANG_LIMIT_COARSE, ANG_GAIN_COARSE * error))
        else:
            # --- rate-limited variant ---
            # One hold may rotate at most COARSE_DEADBAND * 0.5 rad, so the robot
            # physically cannot jump across the deadband it is aiming for.
            max_turn_per_hold = COARSE_DEADBAND * 0.5
            omega_cap = max_turn_per_hold / 0.9  # rad/s, at the default hold
            angular = max(-omega_cap, min(omega_cap, ANG_GAIN_COARSE * error))
            # Creep while turning: with zero linear speed a target more than
            # 90 degrees off-axis can rotate forever without changing geometry.
            linear = min(linear, 0.6)
    else:
        angular = max(-ANG_LIMIT_FINE, min(ANG_LIMIT_FINE, ANG_GAIN_FINE * error))

    return linear, angular


def trial(ros: Rosbridge, hold: float, ticks: int, fixed: bool, urgency: float, wall_risk: float) -> dict:
    ros.reset()
    time.sleep(0.4)
    start = ros.pose()
    start_distance = math.hypot(TARGET["x"] - start["x"], TARGET["y"] - start["y"])
    errors: list[float] = []
    distances: list[float] = []
    commands: list[tuple[float, float]] = []

    for _ in range(ticks):
        hold_until = time.perf_counter() + hold
        while time.perf_counter() < hold_until:
            pose = ros.pose()
            heading = math.atan2(TARGET["y"] - pose["y"], TARGET["x"] - pose["x"])
            error = norm_angle(heading - pose["theta"])
            distance = math.hypot(TARGET["x"] - pose["x"], TARGET["y"] - pose["y"])
            linear, angular = shipped_law(error, urgency, wall_risk, distance, fixed)
            ros.publish(linear, angular)
            errors.append(abs(error))
            distances.append(distance)
            commands.append((round(linear, 2), round(angular, 2)))
            time.sleep(1.0 / ACTUATION_HZ)

    ros.publish(0.0, 0.0)
    time.sleep(0.3)
    end = ros.pose()
    end_distance = math.hypot(TARGET["x"] - end["x"], TARGET["y"] - end["y"])

    # A limit cycle shows up as no net progress: the robot holds its distance
    # while spinning. A controller that keeps closing distance is working even
    # if the heading error is still being worked off, so measure both.
    window = min(60, len(distances) // 3)
    tail_errors = errors[-window:]
    tail_distances = distances[-window:]
    return {
        "mode": "fixed" if fixed else "shipped",
        "start_distance_m": round(start_distance, 2),
        "end_distance_m": round(end_distance, 2),
        "closed_m": round(start_distance - end_distance, 2),
        "initial_error_rad": round(errors[0], 2),
        "median_error_rad": round(sorted(errors)[len(errors) // 2], 2),
        "tail_error_rad": round(sum(tail_errors) / len(tail_errors), 2),
        "final_window_closed_m": round(tail_distances[0] - tail_distances[-1], 2),
        "wall_m": round(distance_to_walls(end["x"], end["y"]), 2),
        "step_log": [
            {"i": i, "err": round(e, 2), "d": round(d, 2), "cmd": c}
            for i, (e, d, c) in enumerate(zip(errors, distances, commands))
        ],
        "sample_commands": commands[:3] + ["..."] + commands[-3:],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--hold", type=float, default=0.9)
    parser.add_argument("--ticks", type=int, default=8)
    parser.add_argument("--urgency", type=float, default=1.0)
    parser.add_argument("--wall-risk", type=float, default=0.2)
    args = parser.parse_args()

    ros = Rosbridge(args.port)
    report: dict = {"hold_s": args.hold, "ticks": args.ticks, "target": TARGET, "trials": []}
    try:
        for fixed in (False, True):
            print(f"running {'FIXED' if fixed else 'SHIPPED'} law ...", flush=True)
            r = trial(ros, args.hold, args.ticks, fixed, args.urgency, args.wall_risk)
            report["trials"].append(r)
            print(json.dumps(r, indent=2))
    finally:
        ros.close()

    print()
    print(f"{'law':<9} {'start d':>8} {'end d':>7} {'closed':>7} {'final win':>10} {'tail err':>9}")
    print("-" * 58)
    for r in report["trials"]:
        print(
            f"{r['mode']:<9} {r['start_distance_m']:>8.2f} {r['end_distance_m']:>7.2f} "
            f"{r['closed_m']:>7.2f} {r['final_window_closed_m']:>10.2f} "
            f"{r['tail_error_rad']:>9.2f}"
        )

    shipped, fixed = report["trials"]
    # A fix is real only if the robot makes sustained progress the shipped law
    # could not. Error settling is a bonus, not the criterion: a rate-limited
    # turn legitimately spends time rotating.
    converged = (
        fixed["closed_m"] > shipped["closed_m"] + 0.3
        and fixed["final_window_closed_m"] > 0.0
    )
    report["verdict"] = (
        "FIXED law closes distance where the shipped law limit-cycles"
        if converged
        else "no improvement measured - do not claim a fix"
    )
    print(f"\n{report['verdict']}")

    out = __import__("pathlib").Path(__file__).resolve().parent.parent / "logs" / "control_law_diagnosis.json"
    out.write_text(json.dumps(report, indent=2))
    print(f"report: {out}")
    return 0 if converged else 1


if __name__ == "__main__":
    raise SystemExit(main())
