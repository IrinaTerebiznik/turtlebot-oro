#!/usr/bin/env bash
# ORO action "Emergency stop": ends a scan and a mission, and stops the wheels.
source /opt/ros/jazzy/setup.bash
# The agent runs actions with a clean env: take the domain from the agent process (compose sets it)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$(tr "\0" "\n" < /proc/1/environ | sed -n "s/^ROS_DOMAIN_ID=//p")}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# The agent may run us with an empty env: ROS needs a log dir
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
pkill -INT -f map_slaming.py || true
# Empty path = the waypoint follower drops its mission
ros2 topic pub --once -w 1 /waypoints nav_msgs/msg/Path '{header: {frame_id: map}}' > /dev/null
# Repeat the stop for 1 s: a single message right after start is lost while DDS is still
# discovering the robot over Wi-Fi
ros2 topic pub -w 1 --max-wait-time-secs 5 -t 10 -r 10 /cmd_vel geometry_msgs/msg/TwistStamped '{}' > /dev/null
echo "Emergency stop: everything stopped"
