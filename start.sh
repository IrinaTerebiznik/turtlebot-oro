#!/usr/bin/env bash
# Starts the fake TurtleBot (+ RViz), and the InOrbit agent if local/agent.env.sh exists.
# Drops you into a ROS shell in the robot container. Exiting the shell stops everything.
set -e
cd "$(dirname "$0")"
xhost +local:docker > /dev/null

if [ -f local/agent.env.sh ]; then
  export COMPOSE_PROFILES=agent
else
  echo "No local/agent.env.sh: starting without the InOrbit agent."
  export COMPOSE_PROFILES=
fi

trap 'docker compose down' EXIT
docker compose up -d --build
docker compose exec turtlebot-sim bash
