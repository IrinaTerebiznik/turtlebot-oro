#!/usr/bin/env python3
# Turns TurtleBot topics into ORO key-values: "key=value" strings on /inorbit/custom_data,
# which the InOrbit agent forwards. Each key matches a DataSourceDefinition in agent/cac/.
# Keys whose topic never publishes (battery, sensor state in the sim) are just not sent.

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String
from turtlebot3_msgs.msg import SensorState

PUBLISH_PERIOD = 1.0       # s
OBSTACLE_HALF_ANGLE = 30   # deg each side of straight ahead
MOVING_SPEED = 0.02        # m/s; faster than this = driving
MOVING_TURN = 0.05         # rad/s; turning faster than this = driving
MOVING_HOLD = 2.0          # s; stay "driving" this long after the last motion
STUCK_AFTER = 20.0         # s of commanded motion...
STUCK_DISTANCE = 0.1       # ...without getting this far (m) = stuck
EXPLORER_NODE = 'map_slammer'   # scripts/map_slaming.py: its running = mapping


def tilt_of(q):
    """Angle (deg) between the robot's up axis and vertical."""
    up_z = 1 - 2 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


class OroKvBridge(Node):
    def __init__(self):
        super().__init__('oro_kv_bridge')
        self.values = {}
        self.pose = None
        self.last_motion = 0.0
        # Stuck watch: when motion was first commanded, and where the robot was then
        self.commanded_since = None
        self.commanded_from = None

        self.kv_pub = self.create_publisher(String, '/inorbit/custom_data', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.on_battery,
                                 qos_profile_sensor_data)
        self.create_subscription(SensorState, '/sensor_state', self.on_sensor_state,
                                 qos_profile_sensor_data)
        # Same message type as the robot's /cmd_vel (TwistStamped in Gazebo and on the real one)
        stamped = self.declare_parameter('stamped', True).value
        self.create_subscription(TwistStamped if stamped else Twist, '/cmd_vel',
                                 self.on_cmd_vel, 10)
        self.create_timer(PUBLISH_PERIOD, self.publish)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)
        twist = msg.twist.twist
        speed = abs(twist.linear.x)
        self.values['speed'] = round(speed, 3)
        if speed > MOVING_SPEED or abs(twist.angular.z) > MOVING_TURN:
            self.last_motion = time.monotonic()

    def on_scan(self, msg):
        # Beams covering the angle (index 0 = straight ahead on TurtleBot lidars)
        n = round(math.radians(OBSTACLE_HALF_ANGLE) / msg.angle_increment)
        front = list(msg.ranges[:n + 1]) + list(msg.ranges[-n:])
        real = [d for d in front if not math.isinf(d) and d != 0.0]
        # Nothing in sight = as far as the laser reaches
        self.values['obstacle_distance'] = round(min(real), 2) if real else msg.range_max

    def on_imu(self, msg):
        self.values['imu_tilt'] = round(tilt_of(msg.orientation), 1)

    def on_battery(self, msg):
        self.values['battery_voltage'] = round(msg.voltage, 2)

    def on_sensor_state(self, msg):
        self.values['sensor_state'] = json.dumps({
            'bumper': msg.bumper,
            'cliff': round(msg.cliff, 2),
            'button': msg.button,
            'torque': msg.torque,
        })

    def on_cmd_vel(self, msg):
        twist = msg.twist if isinstance(msg, TwistStamped) else msg
        commanded = twist.linear.x != 0.0 or twist.angular.z != 0.0
        if not commanded:
            self.commanded_since = None
        elif self.commanded_since is None and self.pose is not None:
            self.commanded_since = time.monotonic()
            self.commanded_from = self.pose

    def stuck(self):
        if self.commanded_since is None or self.pose is None:
            return False
        moved = math.dist(self.pose, self.commanded_from)
        if moved >= STUCK_DISTANCE:
            # Made progress: restart the watch from here
            self.commanded_since = time.monotonic()
            self.commanded_from = self.pose
            return False
        return time.monotonic() - self.commanded_since >= STUCK_AFTER

    def mode(self):
        if EXPLORER_NODE in self.get_node_names():
            return 'mapping'
        if time.monotonic() - self.last_motion < MOVING_HOLD:
            return 'driving'
        return 'idle'

    def publish(self):
        self.values['mode'] = self.mode()
        self.values['stuck'] = 'true' if self.stuck() else 'false'
        for key, value in self.values.items():
            self.kv_pub.publish(String(data=f'{key}={value}'))


def main():
    rclpy.init()
    rclpy.spin(OroKvBridge())


if __name__ == '__main__':
    main()
