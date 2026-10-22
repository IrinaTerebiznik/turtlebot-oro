#!/usr/bin/env bash
cd "$(dirname "$0")"
xhost +local:docker > /dev/null

docker run -it --rm \
  --network host \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  -v "$PWD":/turtlebot-oro \
  turtlebot-oro bash -c "ros2 launch /turtlebot-oro/launch/demo.launch.py > /tmp/launch.log 2>&1 & exec bash"
