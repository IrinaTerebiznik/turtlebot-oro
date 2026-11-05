#!/usr/bin/env bash
# ORO "Waypoints" actions: passes a command to scripts/waypoint_manager.py.
# Args from ORO: command <add|remove_last|clear|visit>
set -e
COMMAND=
while [ $# -ge 2 ]; do
  [ "$1" = command ] && COMMAND=$2
  shift 2
done

source /opt/ros/jazzy/setup.bash
# The agent runs actions with a clean env: take the domain from the agent process (compose sets it)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$(tr "\0" "\n" < /proc/1/environ | sed -n "s/^ROS_DOMAIN_ID=//p")}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# The agent may run us with an empty env: ROS needs a log dir
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
ros2 topic pub --once -w 1 /waypoint_cmd std_msgs/msg/String "{data: $COMMAND}" > /dev/null
echo "Waypoints: $COMMAND"
