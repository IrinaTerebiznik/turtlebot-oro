FROM osrf/ros:jazzy-desktop-full

ENV TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y \
      ros-jazzy-turtlebot3 \
      ros-jazzy-turtlebot3-msgs \
      ros-jazzy-turtlebot3-simulations \
      && rm -rf /var/lib/apt/lists/*