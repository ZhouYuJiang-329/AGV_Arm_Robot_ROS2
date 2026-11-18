#!/usr/bin/env bash
# Read-only acquisition and planning. No real navigation execution nodes.
set -eo pipefail
cd /home/zyj/Code/ROS2/AGV_new/AGV_ROS2_Control_Manager
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=216
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch agv_bringup agv_navigation_real.launch.py mode:=observe domain_id:=216 connection:=192.168.192.5 \
  map:="$PWD/maps/seer/20251120141209997_navigable.yaml" \
  params_file:="$PWD/docs/validation/seer_low_speed_2026-11-13/nav2_locked_arm_site.yaml" \
  localization_params:="$PWD/docs/validation/seer_low_speed_2026-11-13/localization_observe_site.yaml" \
  reuse_lidar:=false reuse_odometry:=false reuse_localization:=false reuse_description:=false rviz:=true
