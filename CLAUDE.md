# m20-sim-to-real

The ROS 2 Humble autonomy stack for the Deep Robotics Lynx M20 (commander with failsafes, mission server, Nav2,
offline natural-language navigation) brought from simulation to the real robot. The locomotion bridge is the only
code that talks to the robot; docs/REAL_ROBOT.md is the plan and the test ladder.

## Run the checks before you claim something works

- `make unit` (or `python3 -m pytest tests -q`): ROS-free logic tests with rclpy shims, seconds, no Docker. CI runs
  them first; every behavior change ships with a test here. The UDP protocol module (`m20_udp.py`) is pure Python
  and fully testable: framing, parsing, the speed table, dead reckoning, the map correction.
- `bash -n` and `shellcheck -S error` on every shell script, `python3 -m compileall -q src tools tests`: CI's
  `static` job. Node code that needs real ROS runs under `colcon test` inside the stack image (`build-and-test`).
- Anything that moves the robot is verified on the robot, step by step, per the ladder in docs/REAL_ROBOT.md,
  with the measured number written into the PR.

## Rules that bind every script and node (docs/ENGINEERING_RULES.md)

1. No silent failure paths: `set -euo pipefail`, no `|| true` or `2>/dev/null` on anything that can fail
   meaningfully, a check that cannot find its input fails with a message.
2. Verify the contract before writing the consumer: message type, publisher QoS (`/m20/mode` is latched), one real
   sample of any sensor data, one real status frame from the robot.
3. Smoke-test every script on tiny input before wiring it into a chain.
4. Two identical environmental failures mean redesign, not a third retry.

## What a review should flag first

- Motion safety: a stop, cancel, crash or timeout path that does not leave the robot at zero velocity (the bridge
  sends three zero axis frames; the robot brakes 0.5 s after the last frame); a goal sent while the previous one
  is still being aborted; a watchdog whose condition cannot become true; an arm check reported as passed without
  reading `/m20/mode`; axis commands sent while the robot is in Navigation usage mode.
- The one-client rule: exactly one UDP socket per robot (`m20_udp.Link`); a second client displaces the first.
  A tool that opens its own Link must not run next to the bridge.
- Stale status: a robot status older than 2 s is absent, never "the last known value"; localization is `ok` only
  when `Location == 0`, never from the confidence field.
- Launchers: background work not stopped on failure paths; success printed after a refused step; checks through
  the `ros2` CLI daemon inside the commander container (it wedges: read ROS state with a small rclpy script).
- Threads: the Link's receiver thread and the ROS timers share the cache only under its lock.
- Duplication: the shared helpers live in `m20_udp.py`; a second framing or parsing copy is a finding.

## Conventions

- Commit messages explain the measured reason for a change (what failed, what number was seen).
- `notes/`, `.claude/` and editor workspace files are never committed. Recorded runs, maps and object maps go to
  `~/m20_sim/...`, not the repository, except the site map pair under `maps/`.
