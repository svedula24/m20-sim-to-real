#!/usr/bin/env bash
# Tear down the navigation layer started by tools/robot/robot_up.sh: Nav2, map_server/AMCL or slam_toolbox,
# pointcloud_to_laserscan, the static transforms, the mission server and the NL navigation container.
# Leaves the compose services (commander, locomotion bridge, station) up: the bridge keeps its watchdog, so a
# robot that was moving stops within 1 s of Nav2 going away. WAITS until everything is dead: a fire-and-forget
# kill let dying Nav2 nodes collide with the next bringup (duplicate action servers answered goals with SUCCEEDED
# while the robot sat parked). SIGINT first so DDS participants unregister cleanly.
set -u
C=${M20_CONTAINER:-docker-commander-1}
PAT="navigation_launch|nav2_smoother|nav2_planner|nav2_behaviors|nav2_bt_navigator|nav2_waypoint|nav2_velocity|nav2_lifecycle|nav2_controller|component_container|slam_toolbox|pointcloud_to_laserscan|static_transform_publisher|mission_server|nav2_map_server|lifecycle_manager_map|nav2_amcl|lifecycle_manager_localization|nav2_collision_monitor|lifecycle_manager_collision"
docker exec "$C" bash -c "
  PIDS=\$(ps -eo pid,args | grep -E '$PAT' | grep -v grep | awk '{print \$1}')
  [ -n \"\$PIDS\" ] && kill -2 \$PIDS 2>/dev/null
  sleep 2
  for i in 1 2 3 4 5 6 7 8; do
    PIDS=\$(ps -eo pid,args | grep -E '$PAT' | grep -v grep | awk '{print \$1}')
    [ -z \"\$PIDS\" ] && break
    kill -9 \$PIDS 2>/dev/null
    sleep 2
  done
  echo \"remaining nav procs: \$(ps -eo args | grep -E '$PAT' | grep -v grep | wc -l)\""
docker rm -f m20_nlnav m20_semmap_rec 2>/dev/null
echo "navigation layer down (compose services still up: docker compose -f docker/compose.yaml down to stop them)"
