FROM osrf/ros:jazzy-desktop-full

ENV TURTLEBOT3_MODEL=burger

RUN apt-get update && apt-get install -y \
      ros-jazzy-turtlebot3 \
      ros-jazzy-turtlebot3-msgs \
      ros-jazzy-turtlebot3-simulations \
      tmux \
      && rm -rf /var/lib/apt/lists/*

# Load ROS in every interactive shell (incl. `docker compose exec`)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
