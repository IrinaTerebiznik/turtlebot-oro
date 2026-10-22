## Run

```bash
./start.sh
```

- Builds the image if needed, starts the fake TurtleBot3 Burger + RViz.
- Starts the InOrbit agent too, if `local/agent.env.sh` exists (copy `local/agent.env.sh.example`).
- Drops you into a ROS shell in the robot container. **Exiting the shell stops everything.**

Logs, from another terminal:

```bash
docker compose logs -f turtlebot-sim   # robot + RViz
docker compose logs -f agent-logs      # InOrbit agent
```

## Drive it

- **Waypoints from RViz:** pick **2D Goal Pose** in the toolbar, click a spot and drag to set the final heading.
- **Keyboard:** in the container shell, run `ros2 run turtlebot3_teleop teleop_keyboard`.

## Files

- `start.sh`: the only entrypoint
- `docker-compose.yml`: robot container + InOrbit agent (`agent` profile)
- `launch/demo.launch.py`: fake robot, robot model, waypoint follower, RViz
- `scripts/waypoint_follower.py`: drives to the last goal on `/goal_pose` (stand-in for Nav2)
- `config/turtlebot.rviz`: TurtleBot3 RViz view on the map, goal tool publishing on `/goal_pose`
- `maps/map.yaml` + `maps/map.png`: sim floor plan (Monopoly board, 5 mm/px ≈ 9.7 m). The robot starts on GO.
- `local/agent.env.sh`: agent config with the robot key (gitignored)
