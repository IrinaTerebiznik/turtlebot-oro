## Run

```bash
./start.sh [-g] [-map] [-k] [-slam]
```

- Builds the image if needed, starts the TurtleBot3 Burger + RViz.
- Starts the InOrbit agent too, if `agent/agent.env.sh` exists (copy `agent/agent.env.sh.example`).
- Drops you into tmux in the robot container. **Exiting tmux stops everything.**

### Flags

All off by default. Combine them, e.g. `./start.sh -g -slam -k`.

| Flag | What it does |
|---|---|
| `-g` | Gazebo robot (`turtlebot3_world`, simulated laser on `/scan`) instead of the fake one. Opens the Gazebo window. |
| `-map` | Serves the Monopoly map (`maps/map.yaml`) and starts the robot on GO. Without it there's no map, and `map` = `odom`. |
| `-k` | Keyboard teleop in its own tmux window. |
| `-slam` | Builds the map live with `slam_toolbox`. Needs `-g`; not with `-map`. |

Without `-g` the robot is the fake TurtleBot: it moves, but has no laser.

### tmux windows

| Window | What's in it |
|---|---|
| `turtlebot` | ROS shell |
| `agent` | Agent log. Ctrl-C leaves a shell. |
| `teleop` | Keyboard teleop (`-k` only). Ctrl-C leaves a shell. |

Switch windows with `Ctrl-b n`.

Logs, from another terminal:

```bash
docker compose logs -f turtlebot-sim   # robot + RViz (+ Gazebo)
docker compose logs -f agent-logs      # InOrbit agent
```

## Drive it

- **Keyboard:** `./start.sh -k`, or in a container shell run `ros2 run turtlebot3_teleop teleop_keyboard`.
- **Waypoints from RViz:** pick **2D Goal Pose** in the toolbar, click a spot and drag to set the final heading.
- **ORO:** joystick in the Navigation dashboard (needs the ORO setup below).

## SLAM

```bash
./start.sh -g -slam -k

## Turtlebot tmux tab
python3 /turtlebot-oro/scripts/map_slaming.py
```

- Drive with ORO teleop, keyboard teleop or RViz goals. The map grows in RViz (every ~5 s) and ORO (~1 min).
- SLAM publishes `map → odom` itself, so the plain `map = odom` link is left out.

**Explore on its own** (instead of teleop), in the `turtlebot` window:

```bash
python3 /turtlebot-oro/scripts/map_slaming.py                       # anywhere
python3 /turtlebot-oro/scripts/map_slaming.py --ros-args -p radius:=3.0   # within 3 m of the start
```

Drives forward, turns toward the more open side when something is closer than 0.5 m ahead,
and heads back when it leaves the radius. Ctrl-C stops it.

## ORO setup
1. Download ORO CLI
```bash
pipx install oro-cli
```
2. Configure your enviroment
```bash
oro context add local --url http://localhost:3000/api --api-key <<<API_KEY>>>
```
3. Apply configs
```bash
oro apply -f agent/cac/ModuleStates.yaml   # agent topics: map, scan, cmd_vel, ...
oro apply -f agent/cac/DashboardDefinition.yaml      # Robot + Navigation dashboards
```
- ModuleStates apply to **every** robot in ORO: use an ORO that only runs TurtleBots.
- The agent picks up changes on restart. Check `local/inorbit_agent.log` for `"cmd_vel_topic":"cmd_vel"`.

**Teleop message type:** the agent copies the type it finds on `cmd_vel`.
Gazebo (`-g`) takes `TwistStamped`, the fake robot `Twist`; the waypoint follower publishes the
matching one (`stamped:=`, set by `-g`), so the agent always detects the right type.

## Files

- `start.sh`: the only entrypoint
- `docker-compose.yml`: robot container + InOrbit agent (`agent` profile)
- `launch/demo.launch.py`: fake or Gazebo robot (`gazebo:=`), map (`map:=`), waypoint follower, RViz
- `scripts/waypoint_follower.py`: drives to the last goal on `/goal_pose` (stand-in for Nav2)
- `scripts/map_slaming.py`: wanders using the laser, optionally within a radius, so SLAM can map
- `config/turtlebot.rviz`: TurtleBot3 RViz view on the map, goal tool publishing on `/goal_pose`
- `maps/map.yaml` + `maps/map.png`: sim floor plan (Monopoly board, 5 mm/px ≈ 9.7 m). The robot starts on GO.
- `agent/agent.env.sh`: agent id, key and ORO URL (gitignored; copy `agent/agent.env.sh.example`)
- `agent/cac/`: ORO config as code (ModuleStates, dashboards)
- `local/`: agent logs (gitignored)
