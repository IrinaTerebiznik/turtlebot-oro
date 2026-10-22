#!/usr/bin/env bash
# Starts the fake TurtleBot (+ RViz), and the InOrbit agent if agent/agent.env.sh exists.
# Drops you into a ROS shell in the robot container. Exiting the shell stops everything.
#   -g    Gazebo robot (simulated laser) instead of the fake one
#   -map  serve the Monopoly map (maps/map.yaml)
#   -k    keyboard teleop in its own tmux window ("teleop")
#   -slam build the map live with slam_toolbox (needs -g or -r, not with -map)
#   -r    real TurtleBot3 on the same network (it runs its own bringup) instead of a simulated one
set -e
cd "$(dirname "$0")"

export GAZEBO=false MAP=false SLAM=false REAL=false
KEYBOARD=false
for arg in "$@"; do
  case $arg in
    -g) GAZEBO=true ;;
    -map) MAP=true ;;
    -k) KEYBOARD=true ;;
    -slam) SLAM=true ;;
    -r) REAL=true ;;
    *) echo "Usage: $0 [-g | -r] [-map] [-k] [-slam]"; exit 1 ;;
  esac
done
if [ "$GAZEBO" = true ] && [ "$REAL" = true ]; then
  echo "-g and -r: pick one robot."; exit 1
fi
if [ "$SLAM" = true ] && [ "$GAZEBO" != true ] && [ "$REAL" != true ]; then
  echo "-slam needs -g or -r: the fake robot has no laser."; exit 1
fi
if [ "$SLAM" = true ] && [ "$MAP" = true ]; then
  echo "-slam and -map both publish /map: pick one."; exit 1
fi

xhost +local:docker > /dev/null

if [ -f agent/agent.env.sh ]; then
  export COMPOSE_PROFILES=agent
else
  echo "No agent/agent.env.sh: starting without the InOrbit agent."
  export COMPOSE_PROFILES=
fi

trap 'docker compose down' EXIT
docker compose up -d --build

# tmux in the robot container: "turtlebot" (ROS shell) + "agent" (agent log, then a shell)
# (+ "teleop" with -k). Window commands run in sh, so they source ROS themselves.
AGENT_LOG=/turtlebot-oro/local/inorbit_agent.log
TMUX_CMD=(tmux new-session -s turtlebot -n turtlebot bash \;
  new-window -n agent "trap : INT; tail -F $AGENT_LOG; exec bash")
if [ "$KEYBOARD" = true ]; then
  TMUX_CMD+=(\; new-window -n teleop
    ". /opt/ros/jazzy/setup.sh; ros2 run turtlebot3_teleop teleop_keyboard; exec bash")
fi
TMUX_CMD+=(\; select-window -t turtlebot)
docker compose exec turtlebot-sim "${TMUX_CMD[@]}"
