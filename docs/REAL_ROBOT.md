# Running the stack on the real Lynx M20

What the robot offers, what this repository does with it, and the order in which to test it. The robot facts
come from Deep Robotics' Software Development Guide (V1.2.1 and the 2025-08 beta) and from a sister project's
runs on two M20s (firmware 1.2.0.1, 2026-08 to 2026-10). Numbers tagged **measured** were observed on a robot;
everything else is the vendor's word or an assumption, and the test ladder below checks each one before the
next step depends on it.

## 1. The robot, as seen from the outside

**Hosts and network.** The robot carries three computers on Ubuntu 20.04 / ROS 2 Foxy: AOS, the motion host
(10.21.31.103: motion control, the inspection protocol, the camera RTSP streams), GOS (10.21.31.104, unused
on the Pro), and NOS, the navigation host (10.21.31.106: localization, mapping, the lidar driver). The two
RoboSense Airy lidars sit on their own subnet (10.21.33.201 front, .202 rear) that an external computer cannot
reach directly. An external computer joins by wire through the aviation-plug Ethernet on the back, with a
static address in 10.21.31.0/24 (this repo uses 10.21.31.100). The robot's Wi-Fi (gateway 10.21.41.1) was never
used for control. Measured UDP round trip on the wire: under 1 ms, no loss.

**Driving: the inspection protocol over UDP.** The robot is a UDP server at 10.21.31.103:30000. Frames carry a
16-byte header (sync `EB 91 EB 90`, ASDU length, message id, format byte 1 = JSON, 7 zero bytes) and one JSON
object `{"PatrolDevice": {"Type", "Command", "Time", "Items"}}`. The client sends a heartbeat (100/100) at
2 Hz; the robot then pushes its status to that client: basic status 1002/6 at 2 Hz (motion state, gait, usage
mode, hard e-stop, charging), motion status 1002/4 at 10 Hz (body velocity LinearX/LinearY/OmegaZ, attitude
roll/pitch/yaw, height), device status 1002/5 at 2 Hz (battery per pack), and navigation + location status
1002/7 at 1 Hz (map pose PosX/PosY/Yaw, Location 0 = normal / 1 = lost). Each request is acknowledged with the
same Type/Command carrying only an ErrorCode, so acknowledgements are told apart by content.

The axis command 2/21 sets X, Y and Yaw as ratios in [-1, 1] of the maximum speed, at 20 Hz, in Regular usage
mode only. The robot brakes about 0.5 s after the last frame. Only one client may send axis commands within
2 s (error E006): the pendant app and the bridge cannot both drive. **Measured** at ratio 1.0 in the Agile Flat
gait: X 2.1 m/s, Y 0.62 m/s, Yaw 1.8 rad/s. **Measured** dead zones below which nothing moves: Y 0.16 m/s,
Yaw 0.15 rad/s. The reported LinearX reads about 0.05 m/s while standing still.

**States.** Usage mode (1101/5): 0 Regular, 1 Navigation, 2 Assist. Motion state (2/22): 0 Idle, 1 Stand,
2 Soft e-stop, 3 Damping, 4 Sit, and 17 "RL Control", the controller that takes axis commands on firmware
1.2.x. Gait (2/23): 0x1001 Basic, 0x3002 Flat (Agile), 0x3003 Stair (Agile). **Measured** quirks: any mode
command resets the gait to Basic; about 3 s after entering RL Control the robot drops the usage mode back to
Regular by itself; standing up can rotate the heading by 0.3 rad; standing on the dock breaks the charging
contacts; the robot sleeps after 30 min unless 1101/6 `Auto:false` is sent.

**Localization and maps.** The robot localizes itself on a map made with its own tools (`drmap mapping` on
NOS, up to 50 x 50 m), and that map is a plain Nav2 map_server pair, `occ_grid.pgm` + `occ_grid.yaml` (0.1 m
cells), under `/var/opt/robot/data/maps/<name>/`. The pose arrives over UDP (1002/7) and on DDS (`/tf`,
one edge `map -> base_link` at 10 Hz). A good map localizes without seeding; 2101/1 seeds it.

**Vendor navigation.** In Navigation usage mode the task 1003/1 drives to a map pose: NavMode 0 "Straight"
(a direct leg) or 1 "Auto" (the vendor planner, Flat gait at Low speed only). **Measured**: straight legs end
within 2-8 cm, Auto legs over 11 m within 5-19 cm; a 10-leg indoor circuit ran unattended. Lessons: the task
ending is not arrival (legs have ended metres short, so arrival is judged on the pose), Auto can hold position
indefinitely, a target a few centimetres away made the robot spin, and map quality dominates everything.

**Sensors over DDS.** The robot's processes are bare Fast DDS applications on domain 0 whose UDP interfaces
are whitelisted to localhost; `sudo /opt/robot/scripts/setup_ros2.sh <your ip>` on AOS and NOS, then a reboot,
opens them (an OTA update reverts it). A Humble node then sees `/IMU` (sensor_msgs/Imu, 200 Hz), `/JOINTS_DATA`
(drdds, 200 Hz), `/ODOM` (a map-frame pose, 10 Hz, not odometry), `/tf`, `/BATTERY_DATA`, `/LOC_BODY_POINTS`
(the localization cloud, base_link, 10 Hz, no floor and nothing within 0.4 m). The vendor's `/LIDAR/POINTS`
never leaves NOS. The raw lidar packets do: enable `multicast-relay.service` on NOS and run `rslidar_sdk` on
the external computer (host address = its own IP, host clock stamps), which gives `/rslidar_points_front` and
`/rslidar_points_rear` at 10 Hz in frames `rslidar_front` / `rslidar_rear`. **Measured** mounts from floor
fits: front (+0.32028, 0, -0.013), rear (-0.32028, 0, -0.013), both pitched -90 degrees, rear yawed 180.

**Cameras.** Two vendor fisheye cameras, H.265 RTSP from AOS (`rtsp://10.21.31.103:8554/video1` and
`/video2`, 800x600, about 30 fps; the numbering follows power-up order). Calibrated as Kannala-Brandt
(cv2.fisheye), front fx 396.0 fy 397.1 cx 386.4 cy 285.0, D [0.0443, 0.0026, -0.0371, 0.0161], 113 x 85
degrees. No depth, no ROS image topic. Vendor URDF mount: (+0.37646, 0, +0.03738), the same numbers the
simulator used.

## 2. What this repository does with it

Everything above the locomotion bridge is the simulator-tested stack, unchanged. The bridge
(`src/m20_locomotion_bridge`) is the only process that talks to the robot:

- **One UDP client** (`m20_udp.Link`): heartbeat at 2 Hz with the status re-polls, one receiver thread into a
  timestamped cache; a status older than 2 s is treated as absent.
- **Velocity**: `/cmd_vel` in m/s is turned into ratios with the measured table (`SpeedTable`, parameters
  `vx_max`, `vy_max`, `wz_max`, re-measure them in step 2) and streamed at 20 Hz. The bridge's 1 s watchdog and
  the robot's own 0.5 s brake both stop the robot if Nav2 or the bridge dies. Disarm sends three zero frames.
- **Odometry**: `odom -> base_link` dead-reckoned from the reported body velocity and attitude yaw
  (`DeadReckoner`, rest bias below 0.06 m/s ignored); `map -> odom` from the robot's own localization
  (`MapCorrection`, exactly what AMCL would publish). Nav2, the mission server and NL navigation see the same
  frames they saw in simulation. `/IMU_DATA` carries roll and pitch to the commander's tip-over failsafe.
- **Vendor navigation as a Nav2 stand-in** (`VendorNav`, parameter `vendor_nav`): serves
  `nav2_msgs/NavigateToPose` on `/navigate_to_pose` with the 1003/1 task, judging arrival on the pose
  (0.35 m), failing on lost localization, 20 s without progress, a vendor error or a timeout. The mission
  server and the NL node need no change.
- **Gaits**: `GaitRequest` STAND / WHEELED (Flat gait, then RL Control) / SIT map to 2/22 and 2/23.

`tools/robot/robot_up.sh` brings it up in two stages. **Stage A** (`M20_NAV=vendor`, the default): compose
services, map_server with the robot's map, vendor navigation, NL navigation on the static map. No lidar driver
on the laptop, no Nav2: the shortest path to "go to the ladder" on the real robot. **Stage B**
(`M20_NAV=nav2`): the same plus `rslidar_sdk` clouds into two `pointcloud_to_laserscan` nodes and Nav2 on the
static map, the configuration the simulator runs. The commander is armed ASSISTED and the mode verified on
`/m20/mode` before the script reports ready.

**The object map needs depth.** The mapping tour (`tools/semmap`) records colour + metric depth with identical
stamps, and the builder places each detection from the depth inside its box. The robot has no depth sensor.
Two ways, in order of effort:

1. An RGB-D camera on the payload rail (USB 3 or GigE are available), driven by its vendor's ROS 2 node remapped
   to `/camera/image_raw`, `/camera/depth` (32FC1 metres) and `/camera/camera_info`, plus one static TF
   `base_link -> camera_optical_link` from a tape measure. Nothing else changes.
2. The vendor fisheye plus the lidar: `tools/camera/fisheye_camera_node.py` rectifies the RTSP stream to a
   pinhole image with the calibrated intrinsics, and `tools/camera/lidar_depth_node.py` projects the front
   lidar cloud into that image as a sparse depth map. Detections then get depth only where lidar points fall
   inside the box, so expect fewer placed objects than in simulation and lower the builder's
   `--min-sightings` to 1 for the first map.

## 3. The test ladder

Each step has a number to write down. Do not start the next until the current one has it. Someone holds the
pendant with a hand on the soft e-stop throughout; the first live drives happen in a cleared 4 x 4 m area at
a 0.3 m/s cap.

| Step | What | Pass when |
|---|---|---|
| 0 | Wire the laptop (10.21.31.100/24), `ping 10.21.31.103`, `python3 tools/robot/m20_probe.py status` with the robot sitting | Basic, motion, location and battery lines print; `loc=ok` on the robot's map |
| 1 | `m20_probe.py stand`, `gait flat`, `control`; sit again | BasicStatus shows state 17 within 10 s, and `sit` lies the robot down |
| 2 | `m20_probe.py drive 0.2 0 0 3` three times, tape measure each run; same for `drive 0 0 0.2 3` with a protractor | Distances agree within 10 %; put the implied vx_max / wz_max into `M20_VX_MAX` / `M20_WZ_MAX` |
| 3 | `make image`, `make up`, bridge log says "M20 answering"; `ros2 topic hz /odom` in the commander container; walk the robot by pendant 2 m and compare `/odom` and `/m20/vendor_pose` | /odom at 10 Hz, both poses move 2 m within 0.1 m, TF map -> base_link follows the robot |
| 4 | Copy the robot's map into `maps/`, `robot_up.sh --map maps/<site>.yaml`, `tools/dev/m20 goto` 2 m ahead | Arrives within 0.35 m; `m20 status` shows ASSISTED; killing the bridge mid-leg stops the robot within 1 s |
| 5 | Camera path chosen above: `ros2 topic hz` on colour and depth, stamps identical, one still checked by eye | 5 Hz or better, depth in metres at a known distance within 5 % |
| 6 | `tools/semmap/record_mapping_run.sh --map maps/<site>.yaml`, then `build_object_map.sh`, inspect `crops/` | At least the objects you can name are in `object_map.json` with sane positions on the map |
| 7 | `robot_up.sh --map ... --object-map ... --scene ...`, `tools/dev/m20 say "go to the ladder" --dry-run`, then live | The dry run names the right object and a reachable approach pose; the live run arrives facing it |
| 8 | Stage B: NOS multicast relay on, `rslidar_sdk` running, `M20_NAV=nav2 robot_up.sh`; repeat step 4 and 7 | Same outcomes with Nav2 driving, costmap shows a person stepping in front of the robot |

Open questions the ladder answers: whether 17 is the controller state on your firmware (step 1), your speed
table (step 2), whether the robot drops to Regular mode on its own in a way that breaks vendor navigation
(step 4), and how much of the object map survives sparse lidar depth (step 6).

## 4. What is deliberately not done

- The joint-level SDK (`sdk_deploy`, `/JOINTS_CMD`): needs a per-robot authorization code, has unverified
  velocity control on hardware, and voids the warranty. Body velocity through the inspection protocol is enough.
- DDS to the robot for control: works, but needs the whitelist edit on two robot hosts that every OTA reverts,
  and a Fast DDS RMW instead of the stack's Cyclone. UDP needs nothing on the robot.
- AMCL on the real robot: the robot's own localization is better than what AMCL would do on a laser scan from
  two hemispherical lidars; keep it as the fallback only if the vendor pose proves unreliable (step 3).

## 5. Rehearsal off the robot: the ladder against BrightAI's protocol emulator (2026-11-05)

Before the first wire-up, steps 0 to 4 and the language layer of step 7 were run against the `basic_server`
emulator that BrightAI keeps in its private repository (`sim/m20_sim_server`, kinematic body, run on this laptop
at 127.0.0.1:30000; nothing here touched a robot). `M20_ROBOT_IP=127.0.0.1` points the stack at it; the blank
`maps/emulator.{pgm,yaml}` (20 x 20 m, no walls) stands in for a site map.

| Step | Result on the emulator | Number |
|---|---|---|
| 0 | PASS | state Sit, `loc=ok`, battery 94 %, usage mode 1 at start |
| 1 | PASS | `stand` reached RL Control (17) at once; `sit` back to 4 |
| 2 | PASS | `drive 0.2 0 0 3`: pose moved 1.27 m -> vx 2.12 m/s at ratio 1.0 (the emulator's table says 2.1); `drive 0 0 0.2 3`: turned 62.5 deg -> wz 1.82 rad/s (table 1.8) |
| 3 | PASS | bridge "M20 answering"; `/odom` 10.0 Hz and `/m20/vendor_pose` 10.2 Hz, both at the emulator's pose |
| 4 | PASS | armed ASSISTED verified on `/m20/mode`; `m20 goto 2 0` -> vendor nav task, "arrived, 0.00 m from the goal" |
| 5, 6 | not possible | the emulator has no cameras; the offline circuit4 object map stood in for step 7 |
| 7 | PASS (language layer) | `m20 say "go to the ladder"`: ladder #28 (match 0.88, four alternatives listed), approach pose (-1.52, -9.23), vendor nav arrived; NL node warm-up 49 s with qwen2.5:7b |

What the rehearsal changed (each a mismatch the robot would also have shown, or a gap the emulator exposed):

- `parse_basic` no longer requires `Gait` and `ControlUsageMode`: the emulator omits `Gait`, and the vendor guide
  lists it but BrightAI's own bridge reads it with a default too. First `status` died on `KeyError: 'Gait'`.
- `m20_probe.py drive` runs without the 1002/4 MotionStatus report (the emulator does not model it): the distance
  then comes from the vendor pose alone; it also prints a `wz_max` estimate for the yaw run.
- The image installs `ros-humble-rmw-cyclonedds-cpp` explicitly: compose selects Cyclone and the fresh image did
  not have it (every container restart-looped).
- The bridge publishes `/odom`, `/m20/vendor_pose` and an identity `map -> odom` from LocationStatus when
  MotionStatus is absent, with one warning; with MotionStatus present (the robot) nothing changed. Without this the
  whole stack looked dead against the emulator (0 Hz everywhere).
- `tools/dev/m20 status` reads the latched topics with rclpy: the `ros2` CLI daemon wedges in the commander
  container and printed "(no mode published)" while the mode was there.

Still unanswered, for the robot: the real speed table (step 2), whether `MotionStatus` arrives at the rate the
dead reckoning assumes (step 3), the camera path (step 5) and the object map from sparse lidar depth (step 6).
