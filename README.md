# m20-sim-to-real

The autonomy stack that was developed and tested in simulation for the Deep Robotics Lynx M20, brought to the
real robot: ROS 2 Humble, a commander with arming and failsafes, a mission server, Nav2, and offline
natural-language navigation ("go to the ladder") that resolves the instruction against a semantic object map
of the site. The simulators stay in the parent project; this repository holds only what runs against the
robot, plus the one piece the simulators never needed: the bridge that talks to the M20 itself.

**Start here:** [`docs/REAL_ROBOT.md`](docs/REAL_ROBOT.md), what the robot offers, what this stack does with it,
and the eight-step test ladder from the bench to "go to the ladder". Then [`CLAUDE.md`](CLAUDE.md) for the checks
every change has to pass.

| Document | For |
|---|---|
| [`docs/REAL_ROBOT.md`](docs/REAL_ROBOT.md) | the robot's interfaces, the design, the test ladder |
| [`docs/SIM_TO_REAL.md`](docs/SIM_TO_REAL.md) | the rules for keeping sim and robot on one artifact |
| [`docs/NAV2_SETUP_RUNBOOK.md`](docs/NAV2_SETUP_RUNBOOK.md), [`docs/LIDAR_RESEARCH.md`](docs/LIDAR_RESEARCH.md) | standing Nav2 up on hardware; the lidar facts |
| [`docs/NL_NAVIGATION.md`](docs/NL_NAVIGATION.md), [`docs/NL_NAVIGATION_DESIGN.md`](docs/NL_NAVIGATION_DESIGN.md) | natural-language navigation: requirements and design |
| [`docs/ENGINEERING_RULES.md`](docs/ENGINEERING_RULES.md) | the four binding rules for code in this repo |
| [`maps/README.md`](maps/README.md) | where the site map comes from and the scene yaml format |

## How it talks to the robot

Everything above the locomotion bridge is unchanged from simulation. The bridge is one UDP client of the
robot's inspection protocol (10.21.31.103:30000): heartbeat, status pushes, and body-velocity axis commands at
20 Hz scaled by a speed table measured on a real M20. It turns the robot's reported body velocity into
`/odom` and the robot's own localization into the `map -> odom` transform, so Nav2 and the mission server see
the frames they saw in simulation. With `vendor_nav` on, it also serves Nav2's `NavigateToPose` action with
the robot's built-in navigation, which lets the whole pipeline run on the robot before any lidar driver or
Nav2 is set up. All of that is `src/m20_locomotion_bridge`; the protocol module has no ROS dependency and is
unit-tested against the vendor's framing.

The robot has no depth sensor. The object map needs one, so `tools/camera` offers the no-new-hardware path
(the vendor fisheye rectified to a pinhole camera, depth projected from the front lidar) and the docs
describe the RGB-D camera alternative.

## Running it

A laptop with an NVIDIA GPU rides with the robot, wired to its back-panel Ethernet at 10.21.31.100. Docker
runs the ROS containers; the robot needs no software changes.

```bash
# bench, no ROS: status, stand, enter the velocity controller, measure the speed table
python3 tools/robot/m20_probe.py status
python3 tools/robot/m20_probe.py stand && python3 tools/robot/m20_probe.py gait flat && python3 tools/robot/m20_probe.py control
python3 tools/robot/m20_probe.py drive 0.2 0 0 3        # 3 s at 20 % forward, tape-measure it

# the stack: images once, then bring it up on the robot's own map
make image && make nlnav-image
bash tools/robot/robot_up.sh --map maps/<site>.yaml      # stage A: vendor navigation under the mission server
tools/dev/m20 goto 2.0 0.0
tools/dev/m20 status

# the object map (needs /camera/image_raw + /camera/depth: tools/camera/camera_up.sh or an RGB-D camera)
bash tools/semmap/record_mapping_run.sh --map maps/<site>.yaml
bash tools/semmap/build_object_map.sh ~/m20_sim/semmap/<site>/run_<ts>

# natural language
bash tools/robot/robot_up.sh --map maps/<site>.yaml --object-map ~/m20_sim/semmap/<site>/object_map.json --scene maps/<site>.scene.yaml
tools/dev/m20 say "go to the ladder" --dry-run
tools/dev/m20 say "go to the ladder"

# stage B: Nav2 on the vendor map with the two RoboSense clouds (rslidar_sdk running on the laptop)
M20_NAV=nav2 bash tools/robot/robot_up.sh --map maps/<site>.yaml
```

Ctrl-C in `robot_up.sh` tears the navigation layer down; `make down` stops the compose services. The bridge's
watchdog and the robot's own 0.5 s brake stop the robot if anything above them dies.

## Tests and CI

```bash
make unit    # ROS-free logic tests (pytest + rclpy shims): protocol, bridge, commander, NL navigation, mapping
make image   # the stack image; make test runs colcon test inside it
```

Every push and pull request runs `.github/workflows/ci.yaml` (`static`, `unit`, `build-and-test`) and the
Claude Code Review workflow. Changes land by pull request into `main`; `CLAUDE.md` is what the reviews apply.

## Layout

- `src/m20_locomotion_bridge` the robot interface: `m20_udp.py` (protocol, speed table, dead reckoning,
  map correction), `bridge_node.py` (the node, the vendor-navigation action server)
- `src/m20_msgs`, `m20_commander`, `m20_missions`, `m20_behaviors`, `m20_station` the stack
- `src/m20_navigation` Nav2 parameters, `pointcloud_to_laserscan` config; `src/m20_perception` EKF config
- `src/m20_nlnav` offline natural-language navigation (object map + LLM + approach poses)
- `tools/robot` `m20_probe.py` (bench tool), `robot_up.sh` (bringup on the robot)
- `tools/camera` fisheye rectifier, lidar-to-depth projection, launcher
- `tools/semmap` mapping tour, keyframe recorder, object-map builder, tour planner
- `tools/nlnav` Ollama and NL navigation launchers; `tools/dev/m20` operator CLI; `tools/nav/kill_stack.sh`
- `docker/` stack image, `m20_nlnav` image (torch + OWLv2), compose; `deploy/` systemd unit
- `maps/` site maps and scene files; `tests/` the unit tests
