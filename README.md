# Working Notes: coupling a System 1 model to a robot control loop

Dated working notes on putting a System 1 (fast, typed-judgment) model inside a ROS 2
robot control loop via an MCP server, with raw evidence and independently verifiable
timestamps.

**Author:** Justin — [github.com/BrewHog](https://github.com/BrewHog) — September 2026

## Status

The individual layers work. The composed closed loop **failed**, reproducibly, and the
failure is documented rather than hidden.

| Layer | Result |
|---|---|
| ROS 2 Jazzy turtlesim in Docker, over rosbridge | reachable — 7 topics, 57 services, 3 nodes |
| MCP client → ros-mcp → rosbridge → robot | PASS — 31 tools, 1.613 m displacement verified independently, 3/3 runs |
| System 1 typed-judgment API | live — 252 ms cold, 146 ms median over 12 ticks |
| Closed loop (judgment → control law → actuation) | FAIL — 0.00 m net progress, rotation limit cycle |

Root cause of the failure is a control-law bug of my own, not the model: a 2.2 rad/s turn
cap held for 0.9 s can rotate up to 1.98 rad while the deadband it aims at is only
0.5 rad wide — roughly 4× overshoot, so the fine-tracking branch is unreachable.

## Layout

```
blog/                    the published site (plain static HTML/CSS)
  index.html             overview, results table, note index
  verify.html            how to verify the timestamps yourself
  notes/                 the seven write-ups
  evidence/              raw per-tick logs, run logs, diagnosis JSON
  src/                   source that produced the evidence
  manifest.txt           SHA-256 of every published file
  manifest.txt.ots       OpenTimestamps proof (Bitcoin-backed)
  manifest.tsr           RFC 3161 timestamp token
scripts/                 runnable harnesses, in the order the notes use them
sim/                     Dockerfile + launch file for the arm64 ROS 2 simulator
```

## Verify the record

```bash
cd blog
shasum -a 256 -c manifest.txt        # 27 files, all OK
pip install opentimestamps-client
ots verify manifest.txt.ots          # Bitcoin-anchored proof for the manifest digest
openssl ts -reply -in manifest.tsr -text   # independent RFC 3161 timestamp
```

`ots verify` reports `Pending confirmation in Bitcoin blockchain` until the next Bitcoin
block contains the calendar commitment (roughly hourly); `ots upgrade manifest.txt.ots`
then promotes it to a confirmed block height. Full details in `blog/verify.html`.

## Attribution

Not mine, and I make no claim to it:

- **[ros-mcp-server](https://github.com/robotmcp/ros-mcp-server)** — Contoro. Used
  unmodified; pinned by commit in `manifest.txt`, not vendored.
- **ROS 2 Jazzy, turtlesim** — Open Robotics / the ROS community.
- **System 1 model (JEV)** — TypeSafe, <https://docs.typesafe.ai/introduction>.
- **OpenTimestamps** — <https://opentimestamps.org>.
- The System 1 / System 2 framing for robotics is a widely discussed idea. I claim no
  ownership of the concept, only of the specific work recorded here.

## What this is not

Not real hardware. Not a benchmark (sample sizes are tens of ticks). Not a claim to have
invented anything. Not safety-tested in any way — a reflex layer with unguarded write
access to `cmd_vel` should not go near real machinery.

## License

Prose and documentation: CC BY 4.0. Code in `scripts/` and `sim/`: MIT.
See `LICENSE`.
