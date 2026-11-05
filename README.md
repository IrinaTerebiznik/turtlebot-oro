## Run

```bash
./start.sh [-g | -r] [-map] [-k] [-slam]
```

Real robot: see [real_tb_onboarding.md](real_tb_onboarding.md).

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
| `-slam` | Builds the map live with `slam_toolbox`. Needs `-g` or `-r`; not with `-map`. |
| `-r` | Real TurtleBot3 on the same network instead of a simulated one ([onboarding](real_tb_onboarding.md)). |

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
- With SLAM on, Nav2 drives (goals, missions, waypoints) and plans around what's on the map.

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
oro apply -f agent/cac      # everything below, or one file at a time:
oro apply -f agent/cac/ModuleStates.yaml           # agent topics: map, scan, cmd_vel, ...
oro apply -f agent/cac/DashboardDefinition.yaml    # Robot + Navigation dashboards
oro apply -f agent/cac/DataSourceDefinition.yaml   # values (see below)
oro apply -f agent/cac/StatusDefinition.yaml
oro apply -f agent/cac/ActionDefinition.yaml       # before incidents: they point at actions
oro apply -f agent/cac/IncidentDefinition.yaml
oro apply -f agent/cac/RobotFootprint.yaml          # Burger-sized circle on the map
```
- ModuleStates apply to **every** robot in ORO: use an ORO that only runs TurtleBots.
- The agent picks up changes on restart. Check `local/inorbit_agent.log` for `"cmd_vel_topic":"cmd_vel"`.

**Teleop message type:** the agent copies the type it finds on `cmd_vel`.
Gazebo (`-g`) takes `TwistStamped`, the fake robot `Twist`; the waypoint follower publishes the
matching one (`stamped:=`, set by `-g`), so the agent always detects the right type.

## In ORO

`scripts/oro_kv_bridge.py` (started by the launch file) turns robot topics into ORO key-values.

| Data source | From | Status / incident |
|---|---|---|
| `batteryVoltage` (real robot) | `/battery_state` | warning < 11.4 V, error < 11.0 V → **Battery low** (error stops the robot) |
| `sensorState` (real robot) | `/sensor_state` | JSON: bumper, cliff, button, torque |
| `imuTilt` | `/imu` | error > 25° → **Lifted** (stops the robot) |
| `speed` | `/odom` | |
| `obstacleDistance` | `/scan`, ±30° ahead | warning < 0.3 m, error < 0.15 m |
| `mode` | idle / driving / mapping (explorer running) | shown, never raised |
| `stuck` | told to move, < 10 cm in 20 s | error → **Stuck** (Emergency stop button) |
| `waypointList` | `waypoint_manager.py` | JSON `[{n, x, y}]` in `map` |
| `nextWaypoint` | `waypoint_manager.py` | `n: (x, y)` where the next tour starts |

| Action | What it does |
|---|---|
| **Scan area** | Runs the explorer in the background: radius 2 / 3 / 5 m or no limit, for 1 / 3 / 5 min. Needs `-g -slam`. |
| **Start mission** | Sends a waypoint route from `config/missions.yaml` (missions 1–3, tuned for the Gazebo world). |
| **Stop mission** | Drops the current route and stops the wheels. |
| **Emergency stop** (Navigation) | Ends any scan or mission and stops the wheels. Incidents use it too. |
| **Add waypoint** (Waypoints) | Saves the robot's current pose. |
| **Remove last waypoint** (Waypoints) | Drops the last one added. |
| **Erase waypoint list** (Waypoints) | Deletes them all (asks to confirm). |
| **Visit waypoints** (Waypoints) | Drives all waypoints in order, starting from the next one; "next" moves on as each is reached. |

Actions run `agent/user_scripts/*.sh` in the agent container. The agent kills scripts after 30 s,
so long jobs start in the background. Logs: `local/scan_area.log`.

## Files

- `start.sh`: the only entrypoint
- `docker-compose.yml`: robot container + InOrbit agent (`agent` profile)
- `launch/demo.launch.py`: fake or Gazebo robot (`gazebo:=`), map (`map:=`), waypoint follower, RViz
- `scripts/waypoint_follower.py`: without `-slam`, drives to the last goal on `/goal_pose` or a route on `/waypoints`; goes around obstacles it sees with the laser (no planner)
- `config/nav2.yaml` + `scripts/nav2_bridge.py`: with `-slam`, Nav2 drives instead and plans around obstacles on the live map; the bridge sends `/waypoints` routes through it
- `scripts/map_slaming.py`: wanders using the laser, optionally within a radius / for a duration, so SLAM can map
- `scripts/oro_kv_bridge.py`: robot topics → ORO key-values on `/inorbit/custom_data`
- `scripts/send_mission.py` + `config/missions.yaml`: missions as waypoint routes on `/waypoints`
- `agent/user_scripts/`: scripts behind ORO's RunScript actions
- `scripts/waypoint_manager.py`: the waypoint list (saved in `local/waypoints.yaml`)
- `config/turtlebot.rviz`: TurtleBot3 RViz view on the map, goal tool publishing on `/goal_pose`
- `maps/map.yaml` + `maps/map.png`: sim floor plan (Monopoly board, 5 mm/px ≈ 9.7 m). The robot starts on GO.
- `agent/agent.env.sh`: agent id, key and ORO URL (gitignored; copy `agent/agent.env.sh.example`)
- `agent/cac/`: ORO config as code (ModuleStates, dashboards)
- `local/`: agent logs (gitignored)
