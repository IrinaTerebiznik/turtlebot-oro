#!/usr/bin/env bash
# ORO action "Stop mission": the waypoint follower drops its route and the wheels stop.
# A running scan keeps going (Emergency stop ends everything).
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# The agent may run us with an empty env: ROS needs a log dir
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
# Empty path = the waypoint follower drops its mission
ros2 topic pub --once -w 1 /waypoints nav_msgs/msg/Path '{header: {frame_id: map}}' > /dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{}' > /dev/null
echo "Mission stopped"
