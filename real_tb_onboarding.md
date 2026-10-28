# Real TurtleBot3 onboarding

Run everything this repo does in simulation (ORO teleop, SLAM mapping, actions, waypoints,
missions, statuses and incidents) on a real **TurtleBot3 Burger with ROS 2 Jazzy**.

## How it fits together

```
 TurtleBot3 (Raspberry Pi)                 Laptop (this repo, ./start.sh -r)
 ─────────────────────────                 ────────────────────────────────────────────
 turtlebot3_bringup:                       InOrbit agent  ──MQTT──►  ORO
   motors, odometry  /odom, /tf            oro_kv_bridge, waypoint_follower,
   lidar             /scan        ◄─ROS 2─►  waypoint_manager, explorer (map_slaming)
   IMU, battery      /imu, /battery_state  slam_toolbox (-slam), RViz
   OpenCR sensors    /sensor_state
   wheels  ◄──────── /cmd_vel (TwistStamped)
```

- The robot runs only the standard TurtleBot3 bringup.
- The laptop runs this repo exactly as in simulation. `-r` swaps the simulated robot for the real
  one on the network.
- Topic names on the real robot match the simulation, so the agent settings, data sources and
  actions work unchanged.

## What carries over, what changes

| | Simulation | Real robot |
|---|---|---|
| ORO config (`agent/cac/`) | applied once per ORO | **same**, nothing to redo |
| Topics | `/scan`, `/odom`, `/imu`, `/cmd_vel` | same names |
| `/cmd_vel` type | `TwistStamped` | `TwistStamped` (Jazzy bringup default) |
| Clock | Gazebo's | real time: **sync the robot and laptop clocks** |
| Battery, sensor state | not published | real values in ORO |
| Missions (`config/missions.yaml`) | tuned for the Gazebo world | **redo for your floor** |

## 1. Set up the robot

Follow the ROBOTIS TurtleBot3 e-Manual "Quick Start Guide" for **ROS 2 Jazzy**: Ubuntu 24.04 and
ROS 2 Jazzy on the Raspberry Pi, the `turtlebot3` packages, and the OpenCR firmware.

On the robot, in `~/.bashrc`:

```bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02        # the lidar on your robot: LDS-01, LDS-02 or LDS-03
export ROS_DOMAIN_ID=30        # any number 0-101; must match the laptop
```

## 2. Network

1. Put the robot and the laptop on the **same Wi-Fi network**. ROS 2 discovery needs multicast,
   which some guest or corporate networks block.
2. Use the **same `ROS_DOMAIN_ID`** on both machines.
3. **Sync the clocks.** SLAM and the agent drop data whose timestamps don't match. On the robot:

   ```bash
   sudo apt install chrony   # then point it at the laptop or a shared NTP server
   chronyc tracking          # offset should be a few ms
   ```

## 3. Start the robot

On the robot:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

## 4. Check the robot from the laptop

Open a ROS shell in the repo's container (or any ROS 2 Jazzy shell) with the same domain:

```bash
export ROS_DOMAIN_ID=30
ros2 topic list                                   # /scan /odom /imu /battery_state /sensor_state /cmd_vel ...
ros2 topic info /cmd_vel -v | grep "Topic type"   # geometry_msgs/msg/TwistStamped
ros2 topic echo /scan --once --field angle_min    # 0.0: index 0 = straight ahead
ros2 topic hz /scan                               # about 5 Hz
```

- **`angle_min` is not 0?** The explorer and the obstacle value assume the first lidar beam points
  straight ahead. Adjust `scripts/map_slaming.py` and `scripts/oro_kv_bridge.py` before driving.
- **No topics?** See Troubleshooting.

## 5. ORO

The ORO config is account-wide: if you applied `agent/cac/` for the simulation, the real robot
already has the dashboards, data sources, statuses, incidents and actions.

1. **Robot identity.** In `agent/agent.env.sh`:
   - Keep `INORBIT_ID` to reuse the simulated robot's entry in ORO, or
   - Set a new one (for example `turtlebot3-real`) to see both robots side by side.
     It needs a robot API key from your ORO admin (`INORBIT_KEY`).
2. **Config not applied yet?** Apply it once (see the README, "ORO setup"):

   ```bash
   oro apply -f agent/cac
   ```

## 6. Run

On the laptop, from this repo:

```bash
ROS_DOMAIN_ID=30 ./start.sh -r            # drive with ORO teleop or RViz goals
ROS_DOMAIN_ID=30 ./start.sh -r -k         # + keyboard teleop window
ROS_DOMAIN_ID=30 ./start.sh -r -slam      # + live mapping
```

`-r` starts no simulated robot: no fake robot, no Gazebo, real-time clock, `TwistStamped`
commands. Everything else is the same as in simulation.

## 7. Use it

**Teleop.** ORO's joystick in the Navigation dashboard, or keyboard with `-k`.

**Map an area.**
1. Start with `-r -slam`.
2. Run the **Scan area** action: begin with *Within 2 m* and *1 minute*, then go bigger.
3. Watch the map grow in RViz (every ~5 s) and ORO (~1 min).
4. Save it from the `turtlebot` tmux window:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f /turtlebot-oro/maps/my_floor
   ```
   SLAM starts a fresh map each run: localizing on a saved map isn't set up in this repo yet.

**Waypoints.** Drive to a spot, run **Add waypoint**. Repeat. **Visit waypoints** tours them in
order. The list survives restarts (`local/waypoints.yaml`), but only means something on the map
it was recorded on.

**Missions.** Routes in `config/missions.yaml` are in map coordinates, tuned for the Gazebo world.
For your floor, record spots with **Add waypoint**, copy the coordinates from the `nextWaypoint` /
`waypointList` values in ORO into `config/missions.yaml`, and update the labels in
`agent/cac/ActionDefinition.yaml` (then `oro apply` it).

**Stop.** **Stop mission** drops the route; **Emergency stop** ends scans and missions and stops
the wheels. The Battery low (error) and Lifted incidents run Emergency stop on their own.

## 8. Real-robot behaviour to expect

| Value | Real robot |
|---|---|
| `batteryVoltage` | 3S LiPo: 12.6 V full. Warning below 11.4 V, error below 11.0 V (stops the robot). Charge before demos. |
| `sensorState` | Bumper, cliff, button, motor torque from the OpenCR board. |
| `imuTilt` | Above 25° = **Lifted**: picking the robot up stops it on purpose. |
| `obstacleDistance` | The lidar can't see closer than 0.12 m, or above/below its scan plane (table tops, low steps). |
| `stuck` | Commanded to move but under 10 cm in 20 s: wheels slipping, blocked by something low. |

## 9. Safety

- The explorer drives at 0.1 m/s and turns away 0.5 m before obstacles; the waypoint follower
  goes up to 0.22 m/s (Burger top speed) **with no obstacle avoidance**.
- Keep **Emergency stop** at hand, and the robot away from stairs and drops: the lidar doesn't see
  edges.
- First drives: an open, flat area, small radius, someone next to the robot.

## Troubleshooting

| What you see | Check |
|---|---|
| `ros2 topic list` on the laptop shows no robot topics | Same `ROS_DOMAIN_ID`, same network, multicast allowed; firewall off for a quick test |
| Robot in ORO but doesn't move with teleop | `ros2 topic info /cmd_vel -v`: one type only (`TwistStamped`); agent log says `Topic 'cmd_vel' uses TwistStamped.` |
| Map smears or jumps; agent log shows TF "extrapolation" errors | Clocks out of sync (`chronyc tracking`) |
| No battery or sensor values in ORO | Robot bringup running? `ros2 topic echo /battery_state --once` |
| Lifted incident on flat ground | `ros2 topic echo /imu --once`: IMU orientation looks wrong → recalibrate per the e-Manual |
| Explorer turns the wrong way or bumps into things ahead | `/scan` `angle_min` not 0 (step 4) |
| Actions do nothing | `local/scan_area.log`, `local/inorbit_agent.log`; the action scripts live in `agent/user_scripts/` |
