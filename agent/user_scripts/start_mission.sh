#!/usr/bin/env bash
# ORO action "Start mission": sends a mission from config/missions.yaml to the waypoint follower.
# Args from ORO: mission <name>
set -e
MISSION=
while [ $# -ge 2 ]; do
  [ "$1" = mission ] && MISSION=$2
  shift 2
done

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# The agent may run us with an empty env: ROS needs a log dir
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
python3 /turtlebot-oro/scripts/send_mission.py "$MISSION"
