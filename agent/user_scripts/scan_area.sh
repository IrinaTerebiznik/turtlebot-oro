#!/usr/bin/env bash
# ORO action "Scan area": explores in the background and returns right away
# (the agent kills scripts after 30 s). Args from ORO: radius <m> duration <s>
# Needs SLAM running (start.sh -g -slam) to actually build a map.
set -e
RADIUS=0 DURATION=180
while [ $# -ge 2 ]; do
  case $1 in
    radius) RADIUS=$2 ;;
    duration) DURATION=$2 ;;
  esac
  shift 2
done

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# The agent may run us with an empty env: ROS needs a log dir
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_logs}
# One explorer at a time
pkill -INT -f map_slaming.py || true
setsid nohup python3 /turtlebot-oro/scripts/map_slaming.py --ros-args \
  -p radius:="$RADIUS" -p duration:="$DURATION" > /root/.inorbit/local/scan_area.log 2>&1 &
echo "Scanning: radius ${RADIUS} m (0 = no limit) for ${DURATION} s"
