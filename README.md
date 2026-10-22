## First time only (and after changing the Dockerfile)

```bash
docker build -t turtlebot-oro .
```

## After that

```bash
./start.sh
```

Opens RViz with a fake TurtleBot3 Burger and leaves you with a shell in the container.
Launch logs go to `/tmp/launch.log` inside the container.

## Drive it

- **Waypoints from RViz:** pick **2D Goal Pose** in the toolbar, click a spot and drag to set the final heading.
- **Keyboard:** in the container shell, run `ros2 run turtlebot3_teleop teleop_keyboard`.

## Files

- `launch/demo.launch.py`: fake robot, robot model, waypoint follower, RViz
- `scripts/waypoint_follower.py`: drives to the last goal on `/goal_pose` (stand-in for Nav2)
- `config/turtlebot.rviz`: TurtleBot3 RViz view, goal tool publishing on `/goal_pose`
