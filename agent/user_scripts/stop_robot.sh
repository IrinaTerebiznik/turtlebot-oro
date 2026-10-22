#!/usr/bin/env bash
# ORO action "Emergency stop": ends a scan and a mission, and stops the wheels.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# The agent may run us with an empty env: ROS needs a log dir
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
pkill -INT -f map_slaming.py || true
# Empty path = the waypoint follower drops its mission
ros2 topic pub --once -w 1 /waypoints nav_msgs/msg/Path '{header: {frame_id: map}}' > /dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{}' > /dev/null
echo "Emergency stop: everything stopped"
