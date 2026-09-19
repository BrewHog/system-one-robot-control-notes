#!/usr/bin/env python3
"""JEV (TypeSafe System One) closed-loop control of the ROS 2 simulator.

This is the project's core hypothesis made concrete: a System 1 model supplies
the semantic judgment, ordinary code owns the control law and the actuation, and
the ROS simulator is the body.

Per tick:
    read /turtle1/pose  ->  ask JEV 3 typed questions  ->  compute cmd_vel  ->  publish

The judgments JEV supplies are the ones that resist a formula (is it worth
detouring for a near wall, and how hard should I commit to the current
waypoint). Steering angles and travel times stay in code, because they are
exact geometry - the split the TypeSafe design guidance calls for.

Usage:
    python3 scripts/jev_controller.py --ticks 20
    python3 scripts/jev_controller.py --ticks 20 --no-jev    # scripted baseline
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path

import websocket
from typesafe_sdk import Choice, Noul, Score, TypeSafeClient

ROOT = Path(__file__).resolve().parent.parent

# Turtlesim's world is 11.09 x 11.09 m. Waypoints are chosen inside that.
WAYPOINTS = [
    {"name": "north-east shelf", "x": 8.6, "y": 8.6},
    {"name": "north-west shelf", "x": 2.4, "y": 8.6},
    {"name": "south-west shelf", "x": 2.4, "y": 2.4},
    {"name": "south-east shelf", "x": 8.6, "y": 2.4},
]

QUESTIONS = {
    "target": Choice(
        instructions=(
            "The robot is driving to shelf waypoints in a warehouse simulator. Which "
            "waypoint should it be heading to next? Prefer a waypoint listed in "
            "`not_yet_reached` so the robot makes progress through the route. It is "
            "mid-journey, so unless the robot is already within one metre of its "
            "current target, keep the current target rather than switching: switching "
            "targets mid-drive makes the robot spin in place. Only choose a different "
            "waypoint when the current one has been reached."
        ),
        criteria={wp["name"]: None for wp in WAYPOINTS},
    ),
    "wall_risk": Noul(
        instructions=(
            "Is the robot close enough to a wall that driving forward at full speed "
            "risks hitting it within about one second?"
        )
    ),
    "urgency": Score(
        instructions=(
            "How much of the available speed should the robot use while it travels "
            "toward its chosen waypoint, given how clear the path ahead is?"
        ),
        # Levels describe concrete situations rather than bare degrees. TypeSafe's
        # guidance is explicit that "move slowly" gives the model nothing to match
        # against, and that each level is judged on its own against the state.
        criteria=[
            "The robot is against an obstacle or misaligned and cannot safely move forward at all",
            "The robot has an obstacle or a wall within about one metre ahead and should creep",
            "The robot has roughly two to three metres of clear space ahead and can travel at a moderate pace",
            "The robot has a long clear straight run to its waypoint and can travel at full speed",
        ],
    ),
}

# A Score runs from 0 to len(criteria) - 1, NOT 0 to 1. Normalizing by the top
# level number is the documented way to put it on 0..1. Clamping a raw score to
# 1.0 instead is not a rounding error: on a 4-level scale it silently rewrites
# every answer to "creep", and because every raw score exceeds 1.0 the channel
# collapses to a constant and the judgment stops reaching the controller.
URGENCY_TOP_LEVEL = len(QUESTIONS["urgency"].criteria) - 1


def normalize_score(raw: float, top_level: int = URGENCY_TOP_LEVEL) -> float:
    """Put a Score on 0..1 the documented way: divide by its top level number."""
    if top_level <= 0:
        return 0.0
    return max(0.0, min(1.0, raw / top_level))

# The judgment selects inside this range; code owns the exact control law.
MIN_LINEAR = 0.0
MAX_LINEAR = 2.2
TURN_GAIN = 1.8
MAX_ANGULAR = 2.2
# Rad/s^2-ish ceiling on how fast the commanded turn rate may change per tick.
MAX_ANGULAR_SLEW = 8.0

# A waypoint counts as reached inside this radius (metres).
ARRIVE_RADIUS = 0.6

# Inside this radius the robot is on top of the waypoint; stop chasing the
# bearing, because at point-blank range the bearing swings wildly and the robot
# spins in place instead of finishing.
DOCK_RADIUS = 0.25

# How often cmd_vel is re-published. turtlesim integrates a decaying command, so
# the command must be held continuously rather than sent once per judgment.
ACTUATION_HZ = 20.0


class Rosbridge:
    """Minimal rosbridge client: subscribe to pose, publish cmd_vel, call services."""

    def __init__(self, port: int) -> None:
        self.ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=10)
        self._id = 0
        # throttle_rate caps the pose stream at ~1/rate ms. Without it turtlesim's
        # publisher floods the socket and every read returns a stale pose from the
        # backlog, which makes the controller think the robot is not moving.
        self.ws.send(
            json.dumps(
                {
                    "op": "subscribe",
                    "topic": "/turtle1/pose",
                    "type": "turtlesim/msg/Pose",
                    "throttle_rate": 0,
                    "queue_length": 1,
                    "id": "pose",
                }
            )
        )

    def next_id(self) -> str:
        self._id += 1
        return f"c{self._id}"

    def pose(self) -> dict:
        """Return the newest pose, discarding any backlog.

        turtlesim publishes /turtle1/pose far faster than the control loop runs.
        If the loop reads one buffered message per cycle it consumes a stale queue
        and the controller sees yesterday's pose, which shows up as a robot that
        spins forever and never closes on its target. Draining to the newest
        message is what makes the feedback current.
        """
        import select as _select

        sock = self.ws.sock
        latest = None
        while True:
            # Only read what is already on the wire.
            ready, _, _ = _select.select([sock], [], [], 0.0 if latest else 2.0)
            if not ready:
                break
            data = json.loads(self.ws.recv())
            if data.get("op") == "publish" and data.get("topic") == "/turtle1/pose":
                latest = data["msg"]
        if latest is None:
            raise TimeoutError("no /turtle1/pose message received")
        return latest

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
                    "id": self.next_id(),
                }
            )
        )

    def set_pen(self, on: bool = True, width: int = 3) -> None:
        self.ws.send(
            json.dumps(
                {
                    "op": "call_service",
                    "service": "/turtle1/set_pen",
                    "args": {"r": 255, "g": 255, "b": 255, "width": width, "off": 0 if on else 1},
                    "id": self.next_id(),
                }
            )
        )

    def close(self) -> None:
        self.publish(0.0, 0.0)
        self.ws.close()


def load_api_key() -> str:
    key = os.environ.get("TYPESAFE_API_KEY")
    if key:
        return key
    for line in (ROOT / ".env").read_text().splitlines():
        if line.strip().startswith("TYPESAFE_API_KEY="):
            return line.split("=", 1)[1].strip()
    raise SystemExit("TYPESAFE_API_KEY not found in env or .env")


def norm_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def distance_to_walls(x: float, y: float) -> float:
    """Metres to the nearest wall in turtlesim's 11.09 m square world."""
    return min(x, y, 11.09 - x, 11.09 - y)


def build_state(
    pose: dict, current_target: str | None, reached: set[str], history: list[str]
) -> dict:
    distances = {
        wp["name"]: round(math.hypot(wp["x"] - pose["x"], wp["y"] - pose["y"]), 2)
        for wp in WAYPOINTS
    }
    unvisited = [wp["name"] for wp in WAYPOINTS if wp["name"] not in reached]
    return {
        "robot": {
            "x": round(pose["x"], 2),
            "y": round(pose["y"], 2),
            "heading_rad": round(pose["theta"], 2),
            "nearest_wall_m": round(distance_to_walls(pose["x"], pose["y"]), 2),
        },
        "waypoints": WAYPOINTS,
        "distance_to_waypoints_m": distances,
        "current_target": current_target,
        "already_reached": sorted(reached),
        "not_yet_reached": unvisited,
    }


def jev_decide(client: TypeSafeClient, state: dict) -> tuple[dict, float]:
    """Ask JEV the three questions. Returns (answers, latency_ms)."""
    started = time.perf_counter()
    response = client.system_one(state=state, questions=QUESTIONS)
    latency_ms = (time.perf_counter() - started) * 1000
    urgency_answer = response.scores["urgency"]
    answers = {
        "target": response.choices["target"].choice,
        "wall_risk": response.nouls["wall_risk"].noul,
        # Keep the raw score and its distribution, and normalize explicitly. The
        # raw value is what a bug in the scaling shows up in, so it is what gets
        # logged; recording only the derived number is how the clamp went unseen.
        "urgency_raw": urgency_answer.score,
        "urgency": normalize_score(urgency_answer.score),
        "urgency_confidence": urgency_answer.confidence,
        "urgency_probabilities": {
            str(k): v for k, v in (urgency_answer.probabilities or {}).items()
        },
    }
    return answers, latency_ms


def scripted_decide(
    pose: dict, current_target: str | None, reached: set[str], history: list[str]
) -> dict:
    """Baseline policy with no model in the loop, for contrast."""
    target = current_target
    if current_target is None or current_target in reached:
        unvisited = [wp for wp in WAYPOINTS if wp["name"] not in reached]
        target = (unvisited or WAYPOINTS)[0]["name"]
    wall = distance_to_walls(pose["x"], pose["y"])
    return {
        "target": target,
        "wall_risk": 1.0 if wall < 1.2 else 0.0,
        "urgency": min(1.0, wall / 2.5),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--ticks", type=int, default=20)
    parser.add_argument(
        "--hold",
        type=float,
        default=0.9,
        help="seconds to hold one JEV judgment before asking again",
    )
    parser.add_argument("--no-jev", action="store_true", help="run the scripted baseline instead")
    parser.add_argument("--capture", default=str(ROOT / "screenshots/jev_run.png"))
    args = parser.parse_args()

    if not args.no_jev:
        os.environ["TYPESAFE_API_KEY"] = load_api_key()

    ros = Rosbridge(args.port)
    ros.set_pen(True)
    history: list[str] = []
    latencies: list[float] = []
    decisions: list[dict] = []

    print(f"{'tick':>4} {'target':<20} {'wall':>5} {'commit':>6} {'lin':>5} {'ang':>6} {'ms':>6}")
    print("-" * 62)

    client_cm = TypeSafeClient() if not args.no_jev else None
    client = client_cm.__enter__() if client_cm else None
    current_target: str | None = None
    reached: set[str] = set()
    stall_ticks = 0
    stall_events: list[dict] = []

    try:
        for tick in range(1, args.ticks + 1):
            # --- Slow loop: observe, then ask JEV what it means. ---
            pose = ros.pose()
            state = build_state(pose, current_target, reached, history)

            if client is not None:
                answers, latency = jev_decide(client, state)
            else:
                answers, latency = scripted_decide(pose, current_target, reached, history), 0.0

            if latency:
                latencies.append(latency)

            target = next(
                (wp for wp in WAYPOINTS if wp["name"] == answers["target"]), WAYPOINTS[0]
            )
            distance = math.hypot(target["x"] - pose["x"], target["y"] - pose["y"])

            # Hysteresis: honour a model switch only when the current target is done
            # or clearly worse. Without this the robot chases a target that changes
            # every tick and spins in place instead of travelling.
            #
            # Arrival is evaluated once, against the target actually being driven.
            # Deciding it earlier - against the pre-switch target - marks unreached
            # waypoints as done and the route silently completes itself.
            if current_target is not None and target["name"] != current_target:
                current_wp = next(wp for wp in WAYPOINTS if wp["name"] == current_target)
                current_distance = math.hypot(
                    current_wp["x"] - pose["x"], current_wp["y"] - pose["y"]
                )
                current_arrived = current_distance < ARRIVE_RADIUS
                # Stay on the current target unless it is done, or the model's pick
                # is strictly closer than what remains.
                if not current_arrived and current_distance <= distance:
                    target = current_wp
                    distance = current_distance

            arrived = distance < ARRIVE_RADIUS
            current_target = target["name"]
            history.append(target["name"])
            if arrived:
                reached.add(current_target)
            all_done = len(reached) >= len(WAYPOINTS)

            # Already normalized to 0..1 by the documented divide-by-top-level.
            # Do NOT clamp the raw score: on this 4-level scale a raw 2.1 means
            # "moderate pace", and clamping it to 1.0 would command "creep".
            urgency = answers["urgency"]
            wall_risk = answers["wall_risk"]

            decisions.append(
                {
                    "tick": tick,
                    "pose": {k: round(pose[k], 3) for k in ("x", "y", "theta")},
                    "state": state,
                    "answers": answers,
                    "distance_to_target": round(distance, 2),
                    "latency_ms": round(latency, 1),
                }
            )
            print(
                f"{tick:>4} {target['name']:<20} d={distance:>4.2f} "
                f"wall={wall_risk:>4.2f} urg={answers['urgency_raw']:>4.2f}"
                f"->{urgency:>4.2f} {latency:>6.0f}ms",
                end="",
                flush=True,
            )

            # --- Fast loop: hold a continuous velocity until the next judgment. ---
            # turtlesim integrates a decaying command, so a single pulse per judgment
            # barely moves it. Re-publishing at ACTUATION_HZ keeps the command live,
            # which is the real shape of a System 1 reflex loop: the model supplies
            # the decision, the code supplies the continuous actuation.
            hold_until = time.perf_counter() + args.hold
            last_linear = last_angular = None
            previous_angular = 0.0
            inner_samples = 0
            cmd_speed_sum = achieved_speed_sum = 0.0
            cmd_spin_sum = achieved_spin_sum = 0.0
            while time.perf_counter() < hold_until:
                cycle_start = time.perf_counter()
                pose = ros.pose()
                heading = math.atan2(target["y"] - pose["y"], target["x"] - pose["x"])
                error = norm_angle(heading - pose["theta"])
                distance = math.hypot(target["x"] - pose["x"], target["y"] - pose["y"])

                linear = MIN_LINEAR + urgency * (MAX_LINEAR - MIN_LINEAR)
                if wall_risk > 0.6:
                    linear = min(linear, 0.45)
                if distance < 0.9:
                    linear = min(linear, 0.6)  # arrive gently

                # Parked on the waypoint (or the whole route is done): hold still
                # rather than servo on a meaningless bearing.
                if distance < DOCK_RADIUS or all_done:
                    ros.publish(0.0, 0.0)
                    last_linear, last_angular = 0.0, 0.0
                    elapsed = time.perf_counter() - cycle_start
                    time.sleep(max(0.0, 1.0 / ACTUATION_HZ - elapsed))
                    continue

                # Proportional unicycle controller with rate limiting. The angular
                # rate is proportional to heading error but clamped and slew-limited,
                # so the robot arcs onto the target. An unclamped or bang-bang turn
                # saturates, overshoots the heading, reverses, and spins in place.
                desired_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, TURN_GAIN * error))
                max_step = MAX_ANGULAR_SLEW * (1.0 / ACTUATION_HZ)
                delta = max(-max_step, min(max_step, desired_angular - previous_angular))
                angular = previous_angular + delta
                previous_angular = angular

                # Slow down while turning hard so the robot converges instead of
                # orbiting at full speed.
                turn_penalty = max(0.15, math.cos(min(abs(error), math.pi / 2)))
                linear *= turn_penalty

                ros.publish(linear, angular)
                last_linear, last_angular = linear, angular
                inner_samples += 1
                cmd_speed_sum += abs(linear)
                achieved_speed_sum += abs(pose.get("linear_velocity", 0.0))
                cmd_spin_sum += abs(angular)
                achieved_spin_sum += abs(pose.get("angular_velocity", 0.0))

                elapsed = time.perf_counter() - cycle_start
                time.sleep(max(0.0, 1.0 / ACTUATION_HZ - elapsed))

            # --- Stall check: pure arithmetic, so it belongs in code, not a model. ---
            # "Commanded to move but not moving" is not representable in the state we
            # send, so without this the loop happily reports progress while the robot
            # is pinned. Comparing commanded against measured velocity is the
            # unambiguous signal, and it costs no judgment call.
            stalled = False
            if inner_samples >= 4:
                cmd_lin = cmd_speed_sum / inner_samples
                got_lin = achieved_speed_sum / inner_samples
                cmd_ang = cmd_spin_sum / inner_samples
                got_ang = achieved_spin_sum / inner_samples
                if cmd_lin > 0.3 and got_lin < 0.15 * cmd_lin:
                    stalled = True
                if cmd_ang > 0.5 and got_ang < 0.15 * cmd_ang:
                    stalled = True
                if stalled:
                    stall_ticks += 1
                    stall_events.append(
                        {
                            "tick": tick,
                            "commanded_linear": round(cmd_lin, 3),
                            "measured_linear": round(got_lin, 3),
                            "commanded_angular": round(cmd_ang, 3),
                            "measured_angular": round(got_ang, 3),
                        }
                    )
            if stall_events:
                decisions[-1]["stall"] = stall_events[-1] if stalled else None
                decisions[-1]["stall_ticks_so_far"] = stall_ticks

            print(
                f"  cmd=({last_linear:.2f},{last_angular:+.2f}) "
                f"-> d={distance:.2f}"
            )

        visited = sorted({d["answers"]["target"] for d in decisions})
        visited = sorted({d["answers"]["target"] for d in decisions})
    finally:
        ros.close()
        if client_cm:
            client_cm.__exit__(None, None, None)

    # Let the simulation settle, then capture whatever it rendered.
    time.sleep(0.5)
    out = ROOT / "logs" / f"jev_run_{int(time.time())}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(
        json.dumps({"decisions": decisions, "stall_events": stall_events}, indent=2)
    )

    print()
    if reached:
        print(f"waypoints reached: {len(reached)}/{len(WAYPOINTS)} -> {sorted(reached)}")
    print(f"stall ticks: {stall_ticks}/{len(decisions)}")
    if latencies:
        print(
            f"JEV latency: n={len(latencies)} min={min(latencies):.0f}ms "
            f"median={statistics.median(latencies):.0f}ms max={max(latencies):.0f}ms"
        )
    print(f"decision log: {out}")

    if args.capture:
        import subprocess

        Path(args.capture).parent.mkdir(exist_ok=True)
        try:
            subprocess.run(
                [str(ROOT / ".venv/bin/vncdotool"), "-s", "127.0.0.1::5900", "capture", args.capture],
                check=True,
                capture_output=True,
                timeout=60,
            )
            print(f"screenshot  : {args.capture}")
        except Exception as exc:  # noqa: BLE001
            print(f"screenshot failed: {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
